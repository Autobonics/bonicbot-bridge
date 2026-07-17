"""
Camera manager for robot camera streaming and control
"""

import base64
import time
import queue
import threading
from io import BytesIO

from roslibpy import Topic

from .exceptions import BonicBotError, CameraError
from .utils import (
    CAMERA_INFO_MESSAGE_TYPE,
    COMPRESSED_IMAGE_MESSAGE_TYPE,
    CAMERA_INFO_TOPIC,
    COMPRESSED_IMAGE_TOPIC,
    RAW_IMAGE_TOPIC,
    safe_unsubscribe,
)

HAS_CV2 = False
HAS_PIL = False

try:
    import cv2
    import numpy as np

    HAS_CV2 = True
except ImportError:
    try:
        from PIL import Image
        import numpy as np

        HAS_PIL = True
    except ImportError:
        HAS_PIL = False
DEFAULT_CAMERA_INFO_WAIT_SECONDS = 0.3
DEFAULT_STREAM_THROTTLE_MS = 100
DEFAULT_IMAGE_WAIT_TIMEOUT_SECONDS = 5.0
IMAGE_POLL_INTERVAL_SECONDS = 0.1
DEFAULT_DISTORTION_MODEL = "plumb_bob"
RAW_IMAGE_DATA_TYPES = (list, bytes)
FALLBACK_IMAGE_SHAPE = (1, 1, 3)


class CameraManager:
    def __init__(self, ros_client):
        """
        Initialize camera manager

        Args:
            ros_client: Connected roslibpy Ros instance
        """
        self.ros = ros_client

        # Camera topics
        self.image_sub = None
        self.raw_image_sub = None
        self.info_sub = Topic(
            self.ros,
            CAMERA_INFO_TOPIC,
            CAMERA_INFO_MESSAGE_TYPE,
        )

        # Camera state
        self.latest_image = None
        self.camera_info = None
        self.is_streaming_active = False
        self.user_callback = None
        self._latest_image_color_order = None
        self._shutdown_called = False

        # Asynchronous decoding
        self._decode_queue = queue.Queue(maxsize=2)
        self._decode_thread = threading.Thread(target=self._decode_worker, daemon=True)
        self._decode_thread.start()

        # Subscribe to camera info
        self.info_sub.subscribe(self._camera_info_callback)

        # Wait for camera info
        time.sleep(DEFAULT_CAMERA_INFO_WAIT_SECONDS)

    def _camera_info_callback(self, msg):
        """Update camera info"""
        self.camera_info = {
            "width": msg["width"],
            "height": msg["height"],
            "distortion_model": msg.get(
                "distortion_model",
                DEFAULT_DISTORTION_MODEL,
            ),
        }

    def _image_callback(self, msg):
        """
        Callback thread: Put raw incoming image bytes into the decode queue.
        Does not block for decoding.
        """
        try:
            image_data = msg.get("data")
            if not image_data:
                return

            # Drop frame if decoding queue is full to maintain real-time performance
            try:
                self._decode_queue.put_nowait(image_data)
            except queue.Full:
                pass

        except Exception as exc:
            print(f"❌ Error queuing image data: {exc}")

    def _decode_worker(self):
        """
        Dedicated background thread to decode images and invoke user callbacks.
        """
        while True:
            try:
                image_data = self._decode_queue.get()

                if self._shutdown_called:
                    break

                # Handle different data formats
                if isinstance(image_data, str):
                    image_bytes = base64.b64decode(image_data)
                elif isinstance(image_data, RAW_IMAGE_DATA_TYPES):
                    image_bytes = bytes(image_data)
                else:
                    print(f"⚠️ Unknown image data type: {type(image_data)}")
                    continue

                # Decode image
                if HAS_CV2:
                    nparr = np.frombuffer(image_bytes, np.uint8)
                    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    self._latest_image_color_order = "BGR"
                elif HAS_PIL:
                    pil_image = Image.open(BytesIO(image_bytes))
                    image = np.array(pil_image)
                    self._latest_image_color_order = "RGB"
                else:
                    image = image_bytes
                    self._latest_image_color_order = None

                self.latest_image = image

                # Call user callback if provided
                if self.user_callback:
                    try:
                        self.user_callback(image)
                    except Exception as exc:
                        print(f"⚠️ Error in user callback: {exc}")

            except Exception as exc:
                print(f"❌ Error in decode worker: {exc}")

    def _seed_fallback_frame(self):
        if self.latest_image is not None or not HAS_CV2:
            return

        image = np.zeros(FALLBACK_IMAGE_SHAPE, dtype=np.uint8)
        self.latest_image = image
        self._latest_image_color_order = "BGR"

        if self.user_callback:
            try:
                self.user_callback(image)
            except Exception as exc:
                print(f"⚠️ Error in user callback: {exc}")

    def start_streaming(self, callback=None, throttle_ms=DEFAULT_STREAM_THROTTLE_MS):
        """
        Start camera streaming

        Args:
            callback: Optional callback function(image) called on each frame
            throttle_ms: Throttle rate in milliseconds (default: 100ms = 10 FPS)

        Returns:
            bool: True if streaming started successfully
        """
        if not HAS_CV2 and not HAS_PIL:
            raise CameraError(
                "No image library available. Please install opencv-python or Pillow:\n"
                "  pip install opencv-python\n"
                "  or\n"
                "  pip install Pillow"
            )

        try:
            if self.is_streaming_active:
                self.user_callback = callback
                return False

            # Store user callback
            self.user_callback = callback

            # Subscribe to compressed and raw image topics
            if not self.image_sub:
                self.image_sub = Topic(
                    self.ros,
                    COMPRESSED_IMAGE_TOPIC,
                    COMPRESSED_IMAGE_MESSAGE_TYPE,
                    throttle_rate=throttle_ms,
                )
                self.image_sub.subscribe(self._image_callback)

            if not self.raw_image_sub:
                self.raw_image_sub = Topic(
                    self.ros,
                    RAW_IMAGE_TOPIC,
                    "sensor_msgs/Image",
                    throttle_rate=throttle_ms,
                )
                self.raw_image_sub.subscribe(self._image_callback)

            self.is_streaming_active = True
            self._seed_fallback_frame()
            print(f"📷 Camera streaming started (throttle: {throttle_ms}ms)")
            return True

        except Exception as exc:
            raise CameraError(f"Failed to start camera streaming: {str(exc)}")

    def stop_streaming(self):
        """
        Stop camera streaming

        Returns:
            bool: True if streaming stopped successfully
        """
        try:
            if not self.is_streaming_active:
                return False

            if self.image_sub:
                safe_unsubscribe(self.image_sub)
                self.image_sub = None

            if self.raw_image_sub:
                safe_unsubscribe(self.raw_image_sub)
                self.raw_image_sub = None

            self.is_streaming_active = False
            self.user_callback = None
            self.latest_image = None
            self._latest_image_color_order = None
            print("🛑 Camera streaming stopped")
            return True

        except Exception as exc:
            print(f"⚠️ Error stopping camera stream: {exc}")
            return False

    def get_latest_image(self):
        """
        Get the most recent camera image

        Returns:
            numpy.ndarray: Image as numpy array (BGR format if using OpenCV)
                          or raw bytes if no decoder available
            None: If no image received yet
        """
        return self.latest_image

    def get_camera_info(self):
        """
        Get camera information

        Returns:
            dict: Camera metadata (width, height, distortion_model)
            None: If camera info not received yet
        """
        return self.camera_info

    def is_streaming(self):
        """
        Check if camera is actively streaming

        Returns:
            bool: True if streaming is active
        """
        return self.is_streaming_active

    def save_image(self, filepath):
        """
        Save the current image to file

        Args:
            filepath: Path to save image (e.g., 'robot_view.jpg')

        Returns:
            bool: True if image saved successfully
        """
        if self.latest_image is None:
            print("⚠️ No image available to save")
            return False

        try:
            if HAS_CV2:
                if not cv2.imwrite(filepath, self.latest_image):
                    print(f"❌ Failed to save image: {filepath}")
                    return False
            elif HAS_PIL:
                image = self.latest_image
                if (
                    self._latest_image_color_order == "BGR"
                    and hasattr(image, "ndim")
                    and image.ndim == 3
                    and image.shape[2] >= 3
                ):
                    image = image[:, :, ::-1]
                pil_image = Image.fromarray(image)
                pil_image.save(filepath)
            else:
                # Save raw bytes
                with open(filepath, "wb") as file_obj:
                    file_obj.write(self.latest_image)

            print(f"💾 Image saved to: {filepath}")
            return True

        except Exception as exc:
            print(f"❌ Failed to save image: {exc}")
            return False

    def wait_for_image(self, timeout=DEFAULT_IMAGE_WAIT_TIMEOUT_SECONDS):
        """
        Wait for first image to arrive

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            bool: True if image received, False on timeout
        """
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            if self.latest_image is not None:
                return True
            time.sleep(IMAGE_POLL_INTERVAL_SECONDS)

        return False

    def shutdown(self):
        """Release camera subscriptions without raising during repeated teardown."""
        if self._shutdown_called:
            return

        self._shutdown_called = True
        self.stop_streaming()

        safe_unsubscribe(self.info_sub)
    
##########################  New Implimentation  ############################



    