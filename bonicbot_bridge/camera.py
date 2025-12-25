"""
Camera manager for robot camera streaming and control
"""

import base64
import time
from io import BytesIO
from roslibpy import Topic, Service, ServiceRequest
from .exceptions import BonicBotError

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    try:
        from PIL import Image
        import numpy as np
        HAS_PIL = True
    except ImportError:
        HAS_PIL = False

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
        self.info_sub = Topic(self.ros, '/camera/camera_info', 'sensor_msgs/CameraInfo')
        
        # Camera services
        self.start_camera_srv = Service(self.ros, '/robot/start_camera', 'std_srvs/Trigger')
        self.stop_camera_srv = Service(self.ros, '/robot/stop_camera', 'std_srvs/Trigger')
        
        # Camera state
        self.latest_image = None
        self.camera_info = None
        self.is_streaming_active = False
        self.user_callback = None
        
        # Subscribe to camera info
        self.info_sub.subscribe(self._camera_info_callback)
        
        # Wait for camera info
        time.sleep(0.3)
    
    def _camera_info_callback(self, msg):
        """Update camera info"""
        self.camera_info = {
            'width': msg['width'],
            'height': msg['height'],
            'distortion_model': msg.get('distortion_model', 'plumb_bob'),
        }
    
    def _image_callback(self, msg):
        """
        Process incoming compressed image
        
        Args:
            msg: CompressedImage message from ROS
        """
        try:
            # Extract image data
            image_data = msg.get('data')
            
            if not image_data:
                return
            
            # Handle different data formats
            if isinstance(image_data, str):
                # Base64 encoded string
                image_bytes = base64.b64decode(image_data)
            elif isinstance(image_data, (list, bytes)):
                # Raw byte array
                image_bytes = bytes(image_data)
            else:
                print(f"⚠️ Unknown image data type: {type(image_data)}")
                return
            
            # Decode image
            if HAS_CV2:
                # Use OpenCV to decode JPEG
                nparr = np.frombuffer(image_bytes, np.uint8)
                image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            elif HAS_PIL:
                # Use Pillow to decode JPEG
                pil_image = Image.open(BytesIO(image_bytes))
                image = np.array(pil_image)
            else:
                # No decoder available, store raw bytes
                image = image_bytes
            
            self.latest_image = image
            
            # Call user callback if provided
            if self.user_callback:
                try:
                    self.user_callback(image)
                except Exception as e:
                    print(f"⚠️ Error in user callback: {e}")
                    
        except Exception as e:
            print(f"❌ Error processing image: {e}")
    
    def start_streaming(self, callback=None, throttle_ms=100):
        """
        Start camera streaming
        
        Args:
            callback: Optional callback function(image) called on each frame
            throttle_ms: Throttle rate in milliseconds (default: 100ms = 10 FPS)
            
        Returns:
            bool: True if streaming started successfully
        """
        if not HAS_CV2 and not HAS_PIL:
            raise BonicBotError(
                "No image library available. Please install opencv-python or Pillow:\n"
                "  pip install opencv-python\n"
                "  or\n"
                "  pip install Pillow"
            )
        
        try:
            # Store user callback
            self.user_callback = callback
            
            # Subscribe to compressed image topic
            if not self.image_sub:
                self.image_sub = Topic(
                    self.ros,
                    '/camera/image_raw/compressed',
                    'sensor_msgs/CompressedImage',
                    throttle_rate=throttle_ms
                )
                self.image_sub.subscribe(self._image_callback)
            
            self.is_streaming_active = True
            print(f"📷 Camera streaming started (throttle: {throttle_ms}ms)")
            return True
            
        except Exception as e:
            raise BonicBotError(f"Failed to start camera streaming: {str(e)}")
    
    def stop_streaming(self):
        """
        Stop camera streaming
        
        Returns:
            bool: True if streaming stopped successfully
        """
        try:
            if self.image_sub:
                self.image_sub.unsubscribe()
                self.image_sub = None
            
            self.is_streaming_active = False
            self.user_callback = None
            print("🛑 Camera streaming stopped")
            return True
            
        except Exception as e:
            print(f"⚠️ Error stopping camera stream: {e}")
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
                cv2.imwrite(filepath, self.latest_image)
            elif HAS_PIL:
                pil_image = Image.fromarray(self.latest_image)
                pil_image.save(filepath)
            else:
                # Save raw bytes
                with open(filepath, 'wb') as f:
                    f.write(self.latest_image)
            
            print(f"💾 Image saved to: {filepath}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to save image: {e}")
            return False
    
    def start_camera_service(self):
        """
        Start camera via ROS service
        
        Returns:
            bool: True if camera service started successfully
        """
        try:
            request = ServiceRequest()
            response = self.start_camera_srv.call(request)
            
            if not response['success']:
                raise BonicBotError(f"Failed to start camera: {response['message']}")
            
            print("📷 Camera service started")
            return True
            
        except Exception as e:
            raise BonicBotError(f"Camera service start failed: {str(e)}")
    
    def stop_camera_service(self):
        """
        Stop camera via ROS service
        
        Returns:
            bool: True if camera service stopped successfully
        """
        try:
            request = ServiceRequest()
            response = self.stop_camera_srv.call(request)
            
            if not response['success']:
                raise BonicBotError(f"Failed to stop camera: {response['message']}")
            
            print("🛑 Camera service stopped")
            return True
            
        except Exception as e:
            raise BonicBotError(f"Camera service stop failed: {str(e)}")
    
    def wait_for_image(self, timeout=5.0):
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
            time.sleep(0.1)
        
        return False
