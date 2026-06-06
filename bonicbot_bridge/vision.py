"""
Vision controller for robot detection mode management.

Manages the vision pipeline by publishing mode-change commands to the
/vision/control topic and tracking the currently active mode via a
live subscription to /robot/vision_mode.  Streams YOLO detection
results from /vision/detections via a throttled background subscriber.

Architecture notes
──────────────────
• Follows the same standalone-class pattern as CameraManager in camera.py.
• roslibpy callbacks fire on their own background threads; there is no
  rclpy executor in this codebase.
• The /vision/control publisher is created once in __init__ and
  advertised immediately — never created per-call.
• The /robot/vision_mode subscriber stays alive for the session;
  get_active_mode() reads self._active_mode directly (no blocking).
"""

import base64
import enum
import json
import os
import subprocess
import threading
import time

from roslibpy import Topic

from .exceptions import BonicBotError
from .utils import (
    STRING_MESSAGE_TYPE,
    COMPRESSED_IMAGE_MESSAGE_TYPE,
    COMPRESSED_IMAGE_TOPIC,
    VISION_CONTROL_TOPIC,
    VISION_MODE_TOPIC,
    VISION_DETECTIONS_TOPIC,
    VISION_FACES_TOPIC,
    VISION_POSE_TOPIC,
    VISION_GESTURE_TOPIC,
    VISION_ARUCO_TOPIC,
    safe_unsubscribe,
    safe_unadvertise,
)

# ── Custom exceptions ──────────────────────────────────────────────────


class VisionError(BonicBotError):
    """Raised on general vision control failures (publish error, not connected)."""

    pass


class DetectionModeError(VisionError):
    """Raised when an invalid DetectionMode is passed to enable_detection()."""

    pass


# ── Detection mode enum ───────────────────────────────────────────────


class DetectionMode(enum.Enum):
    """Valid detection pipeline modes.

    Members:
        FACE   — face detection pipeline
        OBJECT — general object detection (YOLO / SSD)
        LINE   — line-following / lane detection
        YOLO   — YOLO object detection with selectable model
        NONE   — wire signal only; disables the detection pipeline
                 (not a valid target for enable_detection)
    """

    FACE = "face"
    POSE = "pose"
    OBJECT = "object"
    LINE = "line"
    YOLO = "yolo"
    GESTURE = "gesture"
    ARUCO = "aruco"
    NONE = "disable"


# ── Module-level timing constants ──────────────────────────────────────

DEFAULT_DETECTION_THROTTLE_MS = 100  # 10 Hz over WebSocket
DEFAULT_DETECTION_TIMEOUT_S = 5.0


# ── Pipeline constants (from vision_pipeline_node.py — verbatim) ───────

CONFIDENCE_THRESHOLD = 0.35
INFERENCE_TIMER_HZ = 10
DEFAULT_MODEL = "yolo26n"

COCO_KEYPOINT_NAMES = [
    'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
    'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
    'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
    'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
]

HAND_LANDMARK_NAMES = [
    'wrist', 'thumb_cmc', 'thumb_mcp', 'thumb_ip', 'thumb_tip',
    'index_finger_mcp', 'index_finger_pip', 'index_finger_dip', 'index_finger_tip',
    'middle_finger_mcp', 'middle_finger_pip', 'middle_finger_dip', 'middle_finger_tip',
    'ring_finger_mcp', 'ring_finger_pip', 'ring_finger_dip', 'ring_finger_tip',
    'pinky_mcp', 'pinky_pip', 'pinky_dip', 'pinky_tip'
]

DEFAULT_ARUCO_DICT = 'DICT_4X4_50'
DEFAULT_MARKER_SIZE_M = 0.05

GESTURE_MODEL_URL  = 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task'
GESTURE_MODEL_PATH_NAME = 'gesture_recognizer.task'

# Models directory — lives alongside this file
_MODELS_DIR = os.path.join(os.path.dirname(__file__), 'models')


# ── VisionPipeline (pure-Python, roslibpy-based) ──────────────────────


class VisionPipeline:
    """In-process vision inference pipeline using roslibpy.

    Subscribes to the robot's compressed camera topic via the existing
    rosbridge WebSocket, runs YOLO / ArUco / MediaPipe inference locally,
    and publishes structured JSON results back via the same WebSocket.

    This replaces the former rclpy-based ``vision_pipeline_node.py``.
    No ROS2 installation is required on the developer's machine.
    """

    def __init__(self, ros_client):
        """Initialize pipeline (publishers created, but nothing runs yet).

        Args:
            ros_client: Connected roslibpy Ros instance (shared with SDK).
        """
        self.ros = ros_client

        # ── State ───────────────────────────────────────────────────────
        self._active_engine = None   # 'yolo'|'face'|'pose'|'aruco'|'gesture'|None
        self._active_model_name = None
        self._model = None           # ultralytics.YOLO instance
        self._aruco_detector = None
        self._aruco_dict_name = None
        self._gesture_recognizer = None
        self._latest_frame = None    # numpy BGR image
        self._frame_lock = threading.Lock()
        self._running = False
        self._inference_thread = None
        self._shutdown_called = False

        # Approximate BonicBot camera intrinsics
        import numpy as np
        self._np = np
        self._camera_matrix = np.array(
            [[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float32
        )
        self._dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self._calibrated = False

        # Ensure models directory exists
        os.makedirs(_MODELS_DIR, exist_ok=True)

        # ── Result publishers (advertised once, kept for session) ──────
        self._detections_pub = Topic(self.ros, VISION_DETECTIONS_TOPIC, STRING_MESSAGE_TYPE)
        self._detections_pub.advertise()
        self._faces_pub = Topic(self.ros, VISION_FACES_TOPIC, STRING_MESSAGE_TYPE)
        self._faces_pub.advertise()
        self._pose_pub = Topic(self.ros, VISION_POSE_TOPIC, STRING_MESSAGE_TYPE)
        self._pose_pub.advertise()
        self._gesture_pub = Topic(self.ros, VISION_GESTURE_TOPIC, STRING_MESSAGE_TYPE)
        self._gesture_pub.advertise()
        self._aruco_pub = Topic(self.ros, VISION_ARUCO_TOPIC, STRING_MESSAGE_TYPE)
        self._aruco_pub.advertise()
        self._mode_pub = Topic(self.ros, VISION_MODE_TOPIC, STRING_MESSAGE_TYPE)
        self._mode_pub.advertise()

        # ── Camera subscriber (created on start, torn down on stop) ────
        self._camera_sub = None

    # ── Start / Stop / Shutdown ─────────────────────────────────────────

    def start(self, engine, model=DEFAULT_MODEL, dictionary=DEFAULT_ARUCO_DICT):
        """Start (or switch) the inference pipeline.

        Args:
            engine: Engine name ('yolo', 'face', 'pose', 'aruco', 'gesture', 'object').
            model: YOLO model name (used for yolo/object engines).
            dictionary: ArUco dictionary name (used for aruco engine).
        """
        import cv2
        self._cv2 = cv2

        # ── Publish mode to /robot/vision_mode ─────────────────────────
        if engine == "aruco" and dictionary != DEFAULT_ARUCO_DICT:
            payload = f"aruco:{dictionary}"
        elif engine == "yolo":
            payload = f"yolo:{model}"
        else:
            payload = engine
        self._mode_pub.publish({'data': payload})

        # ── Cleanup previous engine resources ──────────────────────────
        self._cleanup_engine(keep_engine=engine)

        # ── Configure engine ───────────────────────────────────────────
        if engine in ("yolo", "object"):
            target_model = model if engine == "yolo" else DEFAULT_MODEL
            if self._active_model_name != target_model or self._model is None:
                self._load_yolo_model(target_model)
            self._active_engine = "yolo"

        elif engine == "face":
            if self._active_model_name != 'yolo26n-face.pt':
                self._load_yolo_model('yolo26n-face.pt')
            self._active_engine = "face"

        elif engine == "pose":
            if self._active_model_name != 'yolo26n-pose.pt':
                self._load_yolo_model('yolo26n-pose.pt')
            self._active_engine = "pose"

        elif engine == "aruco":
            if self._aruco_dict_name != dictionary or self._aruco_detector is None:
                self._load_aruco_detector(dictionary)
            self._active_engine = "aruco"

        elif engine == "gesture":
            if self._gesture_recognizer is None:
                self._load_gesture_recognizer()
            self._active_engine = "gesture"

        else:
            print(f"⚠️ Unknown vision engine '{engine}' — ignoring")
            return

        # ── Subscribe to camera if not already ─────────────────────────
        if self._camera_sub is None:
            self._camera_sub = Topic(
                self.ros,
                COMPRESSED_IMAGE_TOPIC,
                COMPRESSED_IMAGE_MESSAGE_TYPE,
                throttle_rate=DEFAULT_DETECTION_THROTTLE_MS,
            )
            self._camera_sub.subscribe(self._on_camera_frame)

        # ── Start inference thread if not already running ──────────────
        if not self._running:
            self._running = True
            self._inference_thread = threading.Thread(
                target=self._inference_loop, daemon=True
            )
            self._inference_thread.start()

        print(f"🚀 VisionPipeline started: engine={self._active_engine}")

    def stop(self):
        """Stop inference and unsubscribe from camera."""
        self._active_engine = None
        self._running = False

        # Wait for inference thread to finish
        if self._inference_thread is not None:
            self._inference_thread.join(timeout=2.0)
            self._inference_thread = None

        # Unsubscribe from camera
        if self._camera_sub is not None:
            safe_unsubscribe(self._camera_sub)
            self._camera_sub = None

        # Clear frame
        with self._frame_lock:
            self._latest_frame = None

        # Release engine resources
        self._cleanup_engine(keep_engine=None)

        # Publish empty results to all topics
        self._detections_pub.publish({'data': '[]'})
        self._faces_pub.publish({'data': '[]'})
        self._aruco_pub.publish({'data': '[]'})
        self._pose_pub.publish({'data': '[]'})
        self._gesture_pub.publish({'data': '{}'})
        self._mode_pub.publish({'data': 'disable'})

        print("🛑 VisionPipeline stopped")

    def shutdown(self):
        """Full teardown — stop + unadvertise all publishers."""
        if self._shutdown_called:
            return
        self._shutdown_called = True

        self.stop()

        safe_unadvertise(self._detections_pub)
        safe_unadvertise(self._faces_pub)
        safe_unadvertise(self._pose_pub)
        safe_unadvertise(self._gesture_pub)
        safe_unadvertise(self._aruco_pub)
        safe_unadvertise(self._mode_pub)

    def _cleanup_engine(self, keep_engine):
        """Release resources for engines other than keep_engine."""
        if keep_engine not in ("yolo", "face", "pose", "object"):
            self._model = None
            self._active_model_name = None
        if keep_engine != "aruco":
            self._aruco_detector = None
            self._aruco_dict_name = None
        if keep_engine != "gesture" and self._gesture_recognizer is not None:
            try:
                self._gesture_recognizer.close()
            except Exception:
                pass
            self._gesture_recognizer = None

    # ── Camera callback ────────────────────────────────────────────────

    def _on_camera_frame(self, msg):
        """Decode compressed image from rosbridge (base64 JPEG) → BGR numpy."""
        try:
            image_data = msg.get('data')
            if not image_data:
                return

            if isinstance(image_data, str):
                image_bytes = base64.b64decode(image_data)
            elif isinstance(image_data, (list, bytes)):
                image_bytes = bytes(image_data)
            else:
                return

            np = self._np
            cv2 = self._cv2
            nparr = np.frombuffer(image_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                return

            with self._frame_lock:
                self._latest_frame = frame
        except Exception as exc:
            print(f"❌ VisionPipeline: failed to decode camera frame: {exc}")

    # ── Inference loop (10 Hz background thread) ───────────────────────

    def _inference_loop(self):
        """Run inference at INFERENCE_TIMER_HZ on the latest frame."""
        interval = 1.0 / INFERENCE_TIMER_HZ
        while self._running:
            t0 = time.time()

            # Snapshot frame under lock
            with self._frame_lock:
                frame = self._latest_frame

            if frame is not None and self._active_engine is not None:
                try:
                    self._run_inference(frame)
                except Exception as exc:
                    print(f"❌ VisionPipeline inference error: {exc}")

            # Sleep remainder of interval
            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _run_inference(self, frame):
        """Dispatch to the active engine's inference method."""
        import json as _json

        if self._active_engine in ("yolo", "face", "pose"):
            if self._model is None:
                return
            results = self._model(frame, verbose=False)
            if self._active_engine == "yolo":
                detections = self._build_detections(results)
                self._detections_pub.publish({'data': _json.dumps(detections)})
            elif self._active_engine == "face":
                faces = self._build_faces(results)
                self._faces_pub.publish({'data': _json.dumps(faces)})
            elif self._active_engine == "pose":
                pose = self._build_pose_keypoints(results)
                self._pose_pub.publish({'data': _json.dumps(pose)})

        elif self._active_engine == "aruco":
            if self._aruco_detector is None:
                return
            self._run_aruco_detection(frame)

        elif self._active_engine == "gesture":
            if self._gesture_recognizer is None:
                return
            self._run_gesture_recognition(frame)

    # ── Model loading ──────────────────────────────────────────────────

    def _load_yolo_model(self, model_name):
        """Load a YOLO model (lazy ultralytics import)."""
        print(f"📦 Loading YOLO model: {model_name} ...")
        t0 = time.time()

        try:
            from ultralytics import YOLO  # noqa: delayed import
        except ImportError:
            print(
                "❌ ultralytics is not installed! run: pip install bonicbot-bridge[vision]"
            )
            self._model = None
            self._active_model_name = None
            self._active_engine = None
            self._mode_pub.publish({'data': 'disable'})
            return

        try:
            model_path = os.path.join(_MODELS_DIR, model_name)

            if model_name == 'yolo26n-face.pt' and (
                not os.path.exists(model_path) or os.path.getsize(model_path) < 1_000_000
            ):
                print(f"⬇️ Auto-downloading custom face weights: {model_name}...")
                url = f"https://github.com/akanametov/yolo-face/releases/download/1.0.0/{model_name}"
                subprocess.check_call(['wget', '-q', '-O', model_path, url])

            self._model = YOLO(model_path)
            self._active_model_name = model_name
            elapsed = time.time() - t0
            print(f"✅ YOLO model loaded: {model_name} in {elapsed:.2f}s")
        except Exception as exc:
            print(f"❌ Failed to load YOLO model '{model_name}': {exc}")
            self._model = None
            self._active_model_name = None
            self._active_engine = None
            self._mode_pub.publish({'data': 'disable'})

    # ── Results → JSON conversion (verbatim from vision_pipeline_node) ─

    def _build_detections(self, results):
        """Convert ultralytics Results → list[dict] matching the bridge schema."""
        detections = []
        if not results or len(results) == 0:
            return detections

        result = results[0]
        boxes = result.boxes
        names = result.names

        if boxes is None or len(boxes) == 0:
            return detections

        for box in boxes:
            conf = float(box.conf[0])
            if conf < CONFIDENCE_THRESHOLD:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x = int(x1)
            y = int(y1)
            w = int(x2 - x1)
            h = int(y2 - y1)

            cls_id = int(box.cls[0])
            cls_name = names.get(cls_id, f"class_{cls_id}")

            center_x = round(float(x1 + x2) / 2.0, 1)
            center_y = round(float(y1 + y2) / 2.0, 1)

            detections.append({
                "class": cls_name,
                "confidence": round(conf, 3),
                "bbox": [x, y, w, h],
                "center_x": center_x,
                "center_y": center_y,
            })

        return detections

    def _build_faces(self, results):
        """Convert YOLO Face detections to list of dicts."""
        faces = []
        if not results or len(results[0].boxes) == 0 or results[0].keypoints is None:
            return faces

        result = results[0]
        boxes = result.boxes
        keypoints = result.keypoints.xy

        for i, box in enumerate(boxes):
            conf = float(box.conf[0])
            if conf < CONFIDENCE_THRESHOLD:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x = int(x1)
            y = int(y1)
            w = int(x2 - x1)
            h = int(y2 - y1)

            kpts = keypoints[i].tolist()
            landmarks = {}
            if len(kpts) >= 5:
                landmarks = {
                    'nose': [int(kpts[0][0]), int(kpts[0][1])],
                    'left_eye': [int(kpts[1][0]), int(kpts[1][1])],
                    'right_eye': [int(kpts[2][0]), int(kpts[2][1])],
                    'left_ear': [int(kpts[3][0]), int(kpts[3][1])],
                    'right_ear': [int(kpts[4][0]), int(kpts[4][1])]
                }

            faces.append({
                'bbox': [x, y, w, h],
                'confidence': round(conf, 3),
                'landmarks': landmarks
            })

        return faces

    def _build_pose_keypoints(self, results):
        """Convert YOLO Pose landmarks to dict."""
        keypoints = {}
        if not results or len(results[0].boxes) == 0 or results[0].keypoints is None:
            return keypoints

        kpts = results[0].keypoints.xy[0].tolist()
        confs = results[0].keypoints.conf[0].tolist()

        for name, kpt, conf in zip(COCO_KEYPOINT_NAMES, kpts, confs):
            keypoints[name] = {
                'x': int(kpt[0]),
                'y': int(kpt[1]),
                'confidence': round(float(conf), 3)
            }

        return keypoints

    # ── ArUco detection ────────────────────────────────────────────────

    def _load_aruco_detector(self, dict_name):
        """Create an ArUco detector for the given dictionary name."""
        import cv2
        self._cv2 = cv2

        ARUCO_DICT_MAP = {
            'DICT_4X4_50':  cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_5X5_50':  cv2.aruco.DICT_5X5_50,
            'DICT_6X6_50':  cv2.aruco.DICT_6X6_50,
        }

        print(f"📦 Loading ArUco detector: {dict_name} ...")
        if dict_name not in ARUCO_DICT_MAP:
            print(
                f"❌ Unknown ArUco dictionary '{dict_name}'. "
                f"Valid: {list(ARUCO_DICT_MAP)}"
            )
            self._aruco_detector = None
            self._aruco_dict_name = None
            self._active_engine = None
            self._mode_pub.publish({'data': 'disable'})
            return

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_MAP[dict_name])
        aruco_params = cv2.aruco.DetectorParameters()

        aruco_params.adaptiveThreshWinSizeMin  = 3
        aruco_params.adaptiveThreshWinSizeMax  = 23
        aruco_params.adaptiveThreshWinSizeStep = 4
        aruco_params.adaptiveThreshConstant    = 7
        aruco_params.minMarkerPerimeterRate    = 0.02
        aruco_params.maxMarkerPerimeterRate    = 4.0
        aruco_params.polygonalApproxAccuracyRate = 0.05
        aruco_params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
        aruco_params.errorCorrectionRate       = 0.6

        self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        self._aruco_dict_name = dict_name
        print(f"✅ ArUco detector loaded: {dict_name} (tuned params)")

    def _run_aruco_detection(self, frame):
        """Run ArUco marker detection on the given frame."""
        import json as _json
        np = self._np
        cv2 = self._cv2

        try:
            h, w = frame.shape[:2]

            if w != 640 or h != 480:
                scale_x = w / 640.0
                scale_y = h / 480.0
                cam_mtx = self._camera_matrix.copy()
                cam_mtx[0, 0] *= scale_x
                cam_mtx[1, 1] *= scale_y
                cam_mtx[0, 2] = w / 2.0
                cam_mtx[1, 2] = h / 2.0
            else:
                cam_mtx = self._camera_matrix

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._aruco_detector.detectMarkers(gray)
            markers = self._build_aruco_markers(corners, ids, cam_mtx)
            self._aruco_pub.publish({'data': _json.dumps(markers)})
        except Exception as exc:
            print(f"❌ ArUco detection error: {exc}")
            self._aruco_pub.publish({'data': '[]'})

    def _build_aruco_markers(self, corners, ids, camera_matrix=None):
        """Convert ArUco detection output to list of dicts with pose."""
        np = self._np
        cv2 = self._cv2
        markers = []
        if ids is None:
            return markers

        half = DEFAULT_MARKER_SIZE_M / 2.0
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            corner_pts = corners[i].reshape(4, 2)
            center_x = float(np.mean(corner_pts[:, 0]))
            center_y = float(np.mean(corner_pts[:, 1]))

            img_pts = corner_pts.astype(np.float32)
            cam = camera_matrix if camera_matrix is not None else self._camera_matrix
            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_pts, cam, self._dist_coeffs
            )

            if success:
                tvec_list = tvec.flatten().tolist()
                rvec_list = rvec.flatten().tolist()
                distance = float(np.linalg.norm(tvec))
            else:
                tvec_list = [0.0, 0.0, 0.0]
                rvec_list = [0.0, 0.0, 0.0]
                distance = 0.0

            markers.append({
                'id': marker_id,
                'corners': [[int(p[0]), int(p[1])] for p in corner_pts],
                'center_x': round(center_x, 1),
                'center_y': round(center_y, 1),
                'calibrated': self._calibrated,
                'tvec': [round(v, 4) for v in tvec_list],
                'rvec': [round(v, 4) for v in rvec_list],
                'distance_m': round(distance, 4),
            })

        return markers

    # ── Gesture recognition ────────────────────────────────────────────

    def _load_gesture_recognizer(self):
        """Lazy-import MediaPipe Tasks API and init GestureRecognizer."""
        print("📦 Loading Gesture Recognizer ...")
        try:
            import mediapipe as mp  # noqa: delayed import
        except ImportError:
            print(
                "❌ mediapipe is not installed! run: pip install bonicbot-bridge[vision]"
            )
            self._gesture_recognizer = None
            self._active_engine = None
            self._mode_pub.publish({'data': 'disable'})
            return

        try:
            gesture_model_path = os.path.join(_MODELS_DIR, GESTURE_MODEL_PATH_NAME)
            if not os.path.exists(gesture_model_path):
                print(
                    f"⬇️ Auto-downloading gesture model: {GESTURE_MODEL_PATH_NAME} ..."
                )
                subprocess.check_call(['wget', '-q', '-O', gesture_model_path, GESTURE_MODEL_URL])

            base_options = mp.tasks.BaseOptions(
                model_asset_path=gesture_model_path
            )
            options = mp.tasks.vision.GestureRecognizerOptions(
                base_options=base_options,
                running_mode=mp.tasks.vision.RunningMode.IMAGE,
                num_hands=2,
            )
            self._gesture_recognizer = (
                mp.tasks.vision.GestureRecognizer.create_from_options(options)
            )
            print("✅ Gesture Recognizer loaded")
        except Exception as exc:
            print(f"❌ Failed to load Gesture Recognizer: {exc}")
            self._gesture_recognizer = None
            self._active_engine = None
            self._mode_pub.publish({'data': 'disable'})

    def _run_gesture_recognition(self, frame):
        """Run MediaPipe gesture recognition on the given frame."""
        import json as _json

        try:
            import mediapipe as mp

            cv2 = self._cv2
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB, data=rgb
            )
            result = self._gesture_recognizer.recognize(mp_image)
            gesture_data = self._build_gesture_result(result, frame.shape)
            self._gesture_pub.publish({'data': _json.dumps(gesture_data)})
        except Exception as exc:
            print(f"❌ Gesture recognition error: {exc}")
            self._gesture_pub.publish({'data': '{}'})

    def _build_gesture_result(self, result, frame_shape):
        """Convert MediaPipe gesture result to dict."""
        if not result.gestures:
            return {}

        frame_h, frame_w = frame_shape[:2]

        gesture_cat = result.gestures[0][0]
        handedness_cat = result.handedness[0][0]
        landmarks_raw = result.hand_landmarks[0]

        hand_landmarks = []
        for idx, lm in enumerate(landmarks_raw):
            name = HAND_LANDMARK_NAMES[idx] if idx < len(HAND_LANDMARK_NAMES) else f"point_{idx}"
            hand_landmarks.append({
                'name': name,
                'x': int(lm.x * frame_w),
                'y': int(lm.y * frame_h),
            })

        return {
            'gesture': gesture_cat.category_name,
            'confidence': round(float(gesture_cat.score), 3),
            'handedness': handedness_cat.category_name,
            'hand_landmarks': hand_landmarks,
        }


# ── VisionController ──────────────────────────────────────────────────


class VisionController:
    """Standalone vision-pipeline controller — same pattern as CameraManager.

    Instantiated once per session and stored as ``self.vision`` on the
    host ``BonicBot`` object.  Shutdown is called from ``disconnect()``.
    """

    def __init__(self, ros_client):
        """
        Initialize vision controller.

        Args:
            ros_client: Connected roslibpy Ros instance
        """
        self.ros = ros_client

        # Internal state
        self._active_mode = "unknown"
        self._shutdown_called = False

        # ── Shared publisher (created once, never per-call) ────────────
        self._vision_pub = Topic(
            self.ros,
            VISION_CONTROL_TOPIC,
            STRING_MESSAGE_TYPE,
        )
        self._vision_pub.advertise()

        # ── Live mode subscriber (background — stays alive for session) ─
        self._vision_mode_sub = Topic(
            self.ros,
            VISION_MODE_TOPIC,
            STRING_MESSAGE_TYPE,
        )
        self._vision_mode_sub.subscribe(self._on_vision_mode)

        # ── Detections subscriber (throttled — stays alive for session) ──
        self._latest_detections: list[dict] = []
        self._detections_sub = Topic(
            self.ros,
            VISION_DETECTIONS_TOPIC,
            STRING_MESSAGE_TYPE,
            throttle_rate=DEFAULT_DETECTION_THROTTLE_MS,
        )
        self._detections_sub.subscribe(self._on_detections)

        # ── Faces subscriber ─────────────────────────────────────────────
        self._latest_faces: list[dict] = []
        self._faces_sub = Topic(
            self.ros,
            VISION_FACES_TOPIC,
            STRING_MESSAGE_TYPE,
            throttle_rate=DEFAULT_DETECTION_THROTTLE_MS,
        )
        self._faces_sub.subscribe(self._on_faces)

        # ── Pose subscriber ──────────────────────────────────────────────
        self._latest_pose: dict = {}
        self._pose_sub = Topic(
            self.ros,
            VISION_POSE_TOPIC,
            STRING_MESSAGE_TYPE,
            throttle_rate=DEFAULT_DETECTION_THROTTLE_MS,
        )
        self._pose_sub.subscribe(self._on_pose)

        # ── Gesture subscriber ─────────────────────────────────────────────
        self._latest_gesture: dict = {}
        self._gesture_sub = Topic(
            self.ros,
            VISION_GESTURE_TOPIC,
            STRING_MESSAGE_TYPE,
            throttle_rate=DEFAULT_DETECTION_THROTTLE_MS,
        )
        self._gesture_sub.subscribe(self._on_gesture)

        # ── ArUco subscriber ───────────────────────────────────────────────
        # NOTE: throttle_rate=0 (disabled) — the pipeline publishes at 10Hz
        # and rosbridge's throttle at the same rate silently drops messages,
        # causing intermittent missed detections.  No throttle is needed
        # since the 10Hz source rate is already reasonable.
        self._latest_aruco: list[dict] = []
        self._aruco_sub = Topic(
            self.ros,
            VISION_ARUCO_TOPIC,
            STRING_MESSAGE_TYPE,
            throttle_rate=0,
        )
        self._aruco_sub.subscribe(self._on_aruco)

        # ── In-process vision pipeline (replaces vision_pipeline_node) ─
        self._pipeline = VisionPipeline(ros_client)

    # ── Subscriber callbacks ────────────────────────────────────────────

    def _on_vision_mode(self, msg):
        """Update active mode from /robot/vision_mode subscription.

        String assignment is GIL-atomic in CPython — no lock needed for
        this single field.

        Args:
            msg (dict): ROS std_msgs/String message with 'data' key.
        """
        self._active_mode = msg["data"]

    def _on_detections(self, msg):
        """Update latest detections from /vision/detections subscription.

        Parses the incoming JSON string into a list of detection dicts.
        On parse failure, resets to an empty list so stale data is never
        left in place.

        Args:
            msg (dict): ROS std_msgs/String message with 'data' key
                containing a JSON array of detection objects.
        """
        try:
            self._latest_detections = json.loads(msg["data"])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f"⚠️ vision: failed to parse detections: {exc}")
            self._latest_detections = []

    def _on_faces(self, msg):
        """Update latest faces from /vision/faces subscription."""
        try:
            self._latest_faces = json.loads(msg["data"])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f"⚠️ vision: failed to parse faces: {exc}")
            self._latest_faces = []

    def _on_pose(self, msg):
        """Update latest pose from /vision/pose subscription."""
        try:
            self._latest_pose = json.loads(msg["data"])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f"⚠️ vision: failed to parse pose: {exc}")
            self._latest_pose = {}

    def _on_gesture(self, msg):
        """Update latest gesture from /vision/gesture subscription."""
        try:
            self._latest_gesture = json.loads(msg["data"])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f"⚠️ vision: failed to parse gesture: {exc}")
            self._latest_gesture = {}

    def _on_aruco(self, msg):
        """Update latest ArUco markers from /vision/aruco subscription."""
        try:
            self._latest_aruco = json.loads(msg["data"])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f"⚠️ vision: failed to parse aruco: {exc}")
            self._latest_aruco = []

    # ── Public API ─────────────────────────────────────────────────────

    def enable_detection(self, mode, model="yolo26n", dictionary=None):
        """Enable a detection pipeline mode.

        Publishes the mode value to /vision/control via the shared
        publisher that was advertised in __init__.

        For YOLO mode, the payload is ``'yolo:<model>'`` (colon-separated).
        For ARUCO mode, an optional ``dictionary`` parameter can be passed
        to select the ArUco dictionary (default: DICT_4X4_50).

        Args:
            mode (DetectionMode): The detection mode to enable.
            model (str): YOLO model name (default: 'yolo26n').
                         Only used when ``mode`` is DetectionMode.YOLO.
            dictionary (str | None): ArUco dictionary name.
                         Only used when ``mode`` is DetectionMode.ARUCO.

        Returns:
            bool: True on successful publish.

        Raises:
            DetectionModeError: If ``mode`` is not a DetectionMode enum
                member, or if ``mode`` is DetectionMode.NONE.
            VisionError: If the publish call fails.
        """
        # Guard 1: type check — reject raw strings, ints, etc.
        if not isinstance(mode, DetectionMode):
            valid = [m.name for m in DetectionMode if m is not DetectionMode.NONE]
            raise DetectionModeError(
                f"mode must be a DetectionMode enum member, got {type(mode).__name__}. "
                f"Valid modes: {valid}"
            )

        # Guard 2: NONE is reserved for the wire protocol
        if mode is DetectionMode.NONE:
            raise DetectionModeError(
                "use disable_detection() to stop the vision pipeline"
            )

        # Guard 3: LINE mode is defined but not yet implemented
        if mode is DetectionMode.LINE:
            raise DetectionModeError(
                "LINE detection mode is not yet implemented"
            )

        # Build payload
        if mode is DetectionMode.YOLO:
            payload = f"yolo:{model}"
        elif mode is DetectionMode.ARUCO and dictionary is not None:
            payload = f"aruco:{dictionary}"
        else:
            payload = mode.value

        # Publish to /vision/control (for robot-side listeners)
        try:
            self._vision_pub.publish({"data": payload})
        except Exception as exc:
            print(f"⚠️ Failed to publish enable_detection({mode.name}): {exc}")
            raise VisionError(f"Failed to enable detection mode '{payload}': {exc}")

        # Start local inference pipeline
        engine = payload.split(':')[0]
        self._pipeline.start(
            engine=engine,
            model=model,
            dictionary=dictionary or DEFAULT_ARUCO_DICT,
        )

        print(f"👁️ Vision detection enabled: {mode.name} ({payload})")
        return True

    def disable_detection(self):
        """Disable the detection pipeline.

        Publishes DetectionMode.NONE.value ('disable') to
        /vision/control via the shared publisher.

        Returns:
            bool: True on successful publish.

        Raises:
            VisionError: If the publish call fails.
        """
        # Stop local inference pipeline
        self._pipeline.stop()

        try:
            self._vision_pub.publish({"data": DetectionMode.NONE.value})
            print("🛑 Vision detection disabled")
            return True
        except Exception as exc:
            print(f"⚠️ Failed to publish disable_detection: {exc}")
            raise VisionError(f"Failed to disable detection: {exc}")

    def get_active_mode(self):
        """Return the currently active vision mode string.

        Never blocks.  Returns ``'unknown'`` until the first message
        arrives on the /robot/vision_mode subscription.

        Returns:
            str: The active mode string (e.g. 'face', 'object',
                 'disable', or 'unknown').
        """
        return self._active_mode

    def get_detections(self, class_filter=None):
        """Return the latest detection results from the vision pipeline.

        Returns a shallow copy of the internal detections list so callers
        iterating the returned list are not affected by a concurrent
        background callback update.

        Args:
            class_filter (str | None): If given, return only detections
                where ``det['class'] == class_filter``.  If None, return
                all detections.

        Returns:
            list[dict]: Each dict has the schema::

                {
                    'class':      str,    # e.g. 'person', 'bottle'
                    'confidence': float,  # 0.0–1.0
                    'bbox':       [x, y, w, h],  # pixels, top-left origin
                    'center_x':   float,  # pixels
                    'center_y':   float   # pixels
                }

            Returns ``[]`` if no detections have been received yet.
        """
        if class_filter is None:
            return list(self._latest_detections)
        return [d for d in self._latest_detections if d.get("class") == class_filter]

    def get_faces(self) -> list[dict]:
        """Return the latest face detections.
        
        Returns a shallow copy of the internal faces list.
        Returns [] until the first message is received.
        
        Schema (YOLOv8-face):
            [
                {
                    'bbox': [x, y, w, h],
                    'confidence': float,
                    'landmarks': {
                        'nose': [x, y],
                        'left_eye': [x, y],
                        'right_eye': [x, y],
                        'left_ear': [x, y],
                        'right_ear': [x, y]
                    }
                }
            ]
        """
        return list(self._latest_faces)

    def get_pose_keypoints(self) -> dict:
        """Return the latest pose keypoints.
        
        Returns a shallow copy of the internal pose dictionary.
        Returns {} until the first message is received.
        
        Schema (YOLOv8-pose, COCO 17 keypoints):
            {
                'nose': {'x': int, 'y': int, 'confidence': float},
                'left_shoulder': {'x': int, 'y': int, 'confidence': float},
                ... (17 COCO keypoints total, absolute pixel coordinates)
            }
        """
        return dict(self._latest_pose)

    def get_gesture(self) -> 'str | None':
        """Return the current gesture class name, or None if no hand detected.

        Returns:
            str | None: e.g. 'Thumb_Up', 'Open_Palm', 'Victory', or None.
        """
        return self._latest_gesture.get('gesture')

    def get_gesture_full(self) -> dict:
        """Return the full gesture result including landmarks.

        Schema::

            {
                'gesture': str,
                'confidence': float,
                'handedness': str,
                'hand_landmarks': [{'name': str, 'x': int, 'y': int}, ...]
            }

        Returns {} if no hand detected.
        """
        return dict(self._latest_gesture)

    def get_aruco_markers(self) -> list:
        """Return the latest ArUco marker detections.

        Schema (each element)::

            {
                'id': int,
                'corners': [[x,y], [x,y], [x,y], [x,y]],
                'center_x': float, 'center_y': float,
                'calibrated': bool,
                'tvec': [tx, ty, tz],
                'rvec': [rx, ry, rz],
                'distance_m': float
            }

        Returns [] until the first message is received.
        """
        return list(self._latest_aruco)

    def wait_for_marker(self, marker_id: int, timeout: float = 5.0):
        """Block until a specific ArUco marker ID is detected, or timeout.

        Args:
            marker_id: The integer marker ID to look for.
            timeout: Maximum seconds to wait.

        Returns:
            dict | None: The matching marker dict, or None on timeout.
        """
        start = time.time()
        while (time.time() - start) < timeout:
            for m in self.get_aruco_markers():
                if m.get('id') == marker_id:
                    return m
            time.sleep(0.05)
        print(f"⏰ wait_for_marker: ID {marker_id} not found after {timeout}s")
        return None

    def wait_for_detection(self, target_class, timeout=DEFAULT_DETECTION_TIMEOUT_S):
        """Block until a detection of ``target_class`` appears, or timeout.

        Polls ``get_detections(class_filter=target_class)`` in a tight
        loop (50 ms sleep) and returns the first matching detection dict
        as soon as one is found.  Returns ``None`` on timeout.

        Args:
            target_class (str): The 'class' key value to look for
                (e.g. 'person', 'bottle', 'cat').
            timeout (float): Maximum seconds to wait (default:
                ``DEFAULT_DETECTION_TIMEOUT_S``, which is 5.0).

        Returns:
            dict | None: The first matching detection dict, or None if
                the timeout elapses without a match.
        """
        start = time.time()
        while (time.time() - start) < timeout:
            matches = self.get_detections(class_filter=target_class)
            if matches:
                return matches[0]
            time.sleep(0.05)

        print(f"⏰ wait_for_detection: '{target_class}' not found after {timeout}s")
        return None

    def wait_for_gesture(self, gesture_name: str, timeout: float = 5.0):
        """Block until a specific gesture is detected, or timeout.

        Polls ``get_gesture()`` every 50 ms until the current gesture
        matches ``gesture_name`` (case-insensitive).  Returns the full
        gesture dict (from ``get_gesture_full()``) on match.

        Valid gesture names from the pipeline:
            ``'Thumb_Up'``, ``'Open_Palm'``, ``'Pointing_Up'``,
            ``'Thumb_Down'``, ``'Victory'``, ``'ILoveYou'``

        Args:
            gesture_name: The gesture class name to wait for.
            timeout: Maximum seconds to wait (default: 5.0).

        Returns:
            dict | None: The full gesture dict, or None on timeout.
        """
        start = time.time()
        while (time.time() - start) < timeout:
            current = self.get_gesture()
            if current and current.lower() == gesture_name.lower():
                return self.get_gesture_full()
            time.sleep(0.05)
        print(f"⏰ wait_for_gesture: '{gesture_name}' not detected after {timeout}s")
        return None

    def wait_for_face(self, timeout: float = 5.0):
        """Block until any face is detected, or timeout.

        Polls ``get_faces()`` every 50 ms until the list is non-empty.
        Returns the first face dict on success.

        Args:
            timeout: Maximum seconds to wait (default: 5.0).

        Returns:
            dict | None: The first face dict, or None on timeout.
        """
        start = time.time()
        while (time.time() - start) < timeout:
            faces = self.get_faces()
            if faces:
                return faces[0]
            time.sleep(0.05)
        print(f"⏰ wait_for_face: no face detected after {timeout}s")
        return None

    def wait_for_pose(self, timeout: float = 5.0):
        """Block until pose keypoints are detected, or timeout.

        Polls ``get_pose_keypoints()`` every 50 ms until the dict is
        non-empty.  Returns the full keypoints dict on success.

        Args:
            timeout: Maximum seconds to wait (default: 5.0).

        Returns:
            dict | None: The pose keypoints dict, or None on timeout.
        """
        start = time.time()
        while (time.time() - start) < timeout:
            pose = self.get_pose_keypoints()
            if pose:
                return pose
            time.sleep(0.05)
        print(f"⏰ wait_for_pose: no pose detected after {timeout}s")
        return None

    # ── Teardown ───────────────────────────────────────────────────────

    def shutdown(self):
        """Release vision subscriptions and publisher.

        Safe to call multiple times — the ``_shutdown_called`` guard
        ensures idempotency (same pattern as CameraManager.shutdown).
        """
        if self._shutdown_called:
            return

        self._shutdown_called = True

        # Shut down the in-process vision pipeline
        self._pipeline.shutdown()

        # Signal the pipeline to stop — best-effort on shutdown
        try:
            self.disable_detection()
        except VisionError:
            pass

        # Tear down subscriber and publisher
        safe_unsubscribe(self._vision_mode_sub)
        safe_unsubscribe(self._detections_sub)
        safe_unsubscribe(self._faces_sub)
        safe_unsubscribe(self._pose_sub)
        safe_unsubscribe(self._gesture_sub)
        safe_unsubscribe(self._aruco_sub)
        safe_unadvertise(self._vision_pub)
