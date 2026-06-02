#!/usr/bin/env python3
"""
vision_pipeline_node.py — ROS 2 YOLO Vision Pipeline Node (runs on laptop)

Bridges compressed camera frames from the robot through local YOLO inference
and publishes structured JSON detection results back to ROS topics.

Data flow:
    🤖 Robot  →  /camera/image_raw/compressed  →  💻 This node (laptop)
    💻 This node  →  /vision/detections (JSON)  →  🐍 vision.py bridge
    🐍 vision.py  →  /vision/control            →  💻 This node
    💻 This node  →  /robot/vision_mode          →  🐍 vision.py bridge

Subscribed Topics:
    /camera/image_raw/compressed  (sensor_msgs/CompressedImage)
        Raw JPEG frames from robot camera over Wi-Fi.
    /vision/control               (std_msgs/String)
        Commands: 'yolo:yolov8n', 'face', 'line', 'object', 'disable'

Published Topics:
    /vision/detections            (std_msgs/String)
        JSON array of detection dicts (see schema below).
    /robot/vision_mode            (std_msgs/String)
        Current active mode string (echoes parsed command).

Detection JSON schema (each element):
    {
        "class":      str,            # COCO class name e.g. 'person', 'bottle'
        "confidence": float,          # 0.0–1.0, rounded to 3 dp
        "bbox":       [x, y, w, h],   # int pixels, top-left origin
        "center_x":   float,          # pixels
        "center_y":   float           # pixels
    }

Dependencies:
    pip install ultralytics  (lazy-loaded — node starts without it)
    apt: ros-humble-sensor-msgs, ros-humble-std-msgs
    System: opencv-python (cv2), numpy

Run:
    ros2 run bonicbot_bringup vision_pipeline_node
    # or directly:
    python3 vision_pipeline_node.py
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# ── Module-level constants ──────────────────────────────────────────────
CONFIDENCE_THRESHOLD = 0.35
INFERENCE_TIMER_HZ = 10
DEFAULT_MODEL = "yolov8n"
VISION_FACES_TOPIC   = '/vision/faces'
VISION_POSE_TOPIC    = '/vision/pose'
VISION_GESTURE_TOPIC = '/vision/gesture'
VISION_ARUCO_TOPIC   = '/vision/aruco'

COCO_KEYPOINT_NAMES = [
    'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
    'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
    'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
    'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
]

ARUCO_DICT_MAP = {
    'DICT_4X4_50':  cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_5X5_50':  cv2.aruco.DICT_5X5_50,
    'DICT_6X6_50':  cv2.aruco.DICT_6X6_50,
}
DEFAULT_ARUCO_DICT    = 'DICT_4X4_50'
DEFAULT_MARKER_SIZE_M = 0.05

GESTURE_MODEL_URL  = 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task'
GESTURE_MODEL_PATH = 'gesture_recognizer.task'

HAND_LANDMARK_NAMES = [
    'wrist', 'thumb_cmc', 'thumb_mcp', 'thumb_ip', 'thumb_tip',
    'index_finger_mcp', 'index_finger_pip', 'index_finger_dip', 'index_finger_tip',
    'middle_finger_mcp', 'middle_finger_pip', 'middle_finger_dip', 'middle_finger_tip',
    'ring_finger_mcp', 'ring_finger_pip', 'ring_finger_dip', 'ring_finger_tip',
    'pinky_mcp', 'pinky_pip', 'pinky_dip', 'pinky_tip'
]


class VisionPipelineNode(Node):
    """ROS 2 node that subscribes to compressed camera images, runs YOLO
    inference locally on the laptop, and publishes JSON detection results.

    The node decouples inference rate (10 Hz timer) from camera publish
    rate (which may be 30 Hz or higher).  The ``ultralytics`` library is
    lazy-imported only when YOLO inference is first requested so the node
    can start cleanly even if the library is not installed.
    """

    def __init__(self):
        super().__init__("vision_pipeline_node")

        # ── State ───────────────────────────────────────────────────────
        self._active_engine = None  # 'yolo'|'face'|'pose'|'aruco'|'gesture'|None
        self._active_model_name = None  # e.g. 'yolov8n', 'yolov8s'
        self._model = None  # ultralytics.YOLO instance
        self._aruco_detector = None      # cv2.aruco.ArucoDetector instance
        self._aruco_dict_name = None     # e.g. 'DICT_4X4_50' — guard re-init
        self._gesture_recognizer = None  # mp.tasks.vision.GestureRecognizer
        self._latest_frame = None  # numpy BGR image

        # Approximate BonicBot camera intrinsics — replace with calibrated values
        self._camera_matrix = np.array(
            [[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float32
        )
        self._dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self._calibrated = False  # flips True only when real intrinsics loaded

        # ── Subscribers ─────────────────────────────────────────────────
        self._control_sub = self.create_subscription(
            String,
            "/vision/control",
            self._control_callback,
            10,
        )

        self._image_sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self._image_callback,
            10,
        )

        # ── Publishers ──────────────────────────────────────────────────
        self._detections_pub = self.create_publisher(String, "/vision/detections", 10)
        self._faces_pub = self.create_publisher(String, VISION_FACES_TOPIC, 10)
        self._pose_pub = self.create_publisher(String, VISION_POSE_TOPIC, 10)
        self._gesture_pub = self.create_publisher(String, VISION_GESTURE_TOPIC, 10)
        self._aruco_pub = self.create_publisher(String, VISION_ARUCO_TOPIC, 10)
        self._mode_pub = self.create_publisher(String, "/robot/vision_mode", 10)

        # ── Inference timer (10 Hz) ────────────────────────────────────
        self._inference_timer = self.create_timer(
            1.0 / INFERENCE_TIMER_HZ, self._inference_timer_callback
        )

        self.get_logger().info(
            "🚀 VisionPipelineNode started — waiting for /vision/control commands"
        )

    # ── /vision/control callback ────────────────────────────────────────

    def _control_callback(self, msg: String):
        """Parse a control command and update the active engine/model.

        Command format examples:
            'yolo:yolov8n'  → engine='yolo', model='yolov8n'
            'yolo:yolov8s'  → engine='yolo', model='yolov8s'
            'disable'       → stop all inference
            'face'          → future mode (log warning, don't crash)
        """
        raw = msg.data.strip()
        if not raw:
            self.get_logger().warn("Received empty /vision/control command — ignoring")
            return

        parts = raw.split(":")
        engine = parts[0]
        model = parts[1] if len(parts) > 1 else DEFAULT_MODEL

        self.get_logger().info(f"📥 /vision/control received: '{raw}'")

        # Publish mode to /robot/vision_mode IMMEDIATELY — before model load
        mode_msg = String()
        mode_msg.data = raw
        self._mode_pub.publish(mode_msg)

        # ── Handle 'disable' ───────────────────────────────────────────
        if engine == "disable":
            self._active_engine = None
            self._active_model_name = None
            self._model = None
            self._aruco_detector = None
            self._aruco_dict_name = None
            if self._gesture_recognizer is not None:
                try:
                    self._gesture_recognizer.close()
                except Exception:
                    pass
                self._gesture_recognizer = None
            # Publish empty to all result topics
            empty_list = String()
            empty_list.data = "[]"
            self._detections_pub.publish(empty_list)
            self._faces_pub.publish(empty_list)
            self._aruco_pub.publish(empty_list)
            self._pose_pub.publish(empty_list)
            empty_dict = String()
            empty_dict.data = "{}"
            self._gesture_pub.publish(empty_dict)
            self.get_logger().info("🛑 Vision pipeline disabled")
            return

        # ── Handle 'yolo' ──────────────────────────────────────────────
        if engine == "yolo":
            # Cleanup: release non-YOLO engines
            self._aruco_detector = None
            self._aruco_dict_name = None
            if self._gesture_recognizer is not None:
                try:
                    self._gesture_recognizer.close()
                except Exception:
                    pass
                self._gesture_recognizer = None
            # Check if we need to load or swap the model
            if self._active_model_name != model or self._model is None:
                if (
                    self._active_model_name is not None
                    and self._active_model_name != model
                ):
                    self.get_logger().info(
                        f"🔄 Switching YOLO model: {self._active_model_name} → {model}"
                    )
                self._load_model(model)
            self._active_engine = "yolo"
            return

        # ── Handle 'face' ──────────────────────────────────────────────
        if engine == "face":
            # Cleanup: release non-YOLO engines
            self._aruco_detector = None
            self._aruco_dict_name = None
            if self._gesture_recognizer is not None:
                try:
                    self._gesture_recognizer.close()
                except Exception:
                    pass
                self._gesture_recognizer = None
            if self._active_model_name != 'yolov8n-face.pt':
                self._load_model('yolov8n-face.pt')
            self._active_engine = "face"
            return

        # ── Handle 'pose' ──────────────────────────────────────────────
        if engine == "pose":
            # Cleanup: release non-YOLO engines
            self._aruco_detector = None
            self._aruco_dict_name = None
            if self._gesture_recognizer is not None:
                try:
                    self._gesture_recognizer.close()
                except Exception:
                    pass
                self._gesture_recognizer = None
            if self._active_model_name != 'yolov8n-pose.pt':
                self._load_model('yolov8n-pose.pt')
            self._active_engine = "pose"
            return

        # ── Handle 'aruco' ─────────────────────────────────────────────
        if engine == "aruco":
            dict_name = parts[1] if len(parts) > 1 else DEFAULT_ARUCO_DICT
            # Cleanup: release YOLO model and gesture recognizer
            self._model = None
            self._active_model_name = None
            if self._gesture_recognizer is not None:
                try:
                    self._gesture_recognizer.close()
                except Exception:
                    pass
                self._gesture_recognizer = None
            if self._aruco_dict_name != dict_name or self._aruco_detector is None:
                self._load_aruco_detector(dict_name)
            self._active_engine = "aruco"
            return

        # ── Handle 'gesture' ───────────────────────────────────────────
        if engine == "gesture":
            # Cleanup: release YOLO model and aruco detector
            self._model = None
            self._active_model_name = None
            self._aruco_detector = None
            self._aruco_dict_name = None
            if self._gesture_recognizer is None:
                self._load_gesture_recognizer()
            self._active_engine = "gesture"
            return

        # ── Unknown / future engines ───────────────────────────────────
        self.get_logger().warn(
            f"⚠️ Unknown vision engine '{engine}' — not implemented yet. "
            f"Ignoring command '{raw}'."
        )

    # ── /camera/image_raw/compressed callback ──────────────────────────

    def _image_callback(self, msg: CompressedImage):
        """Decode a CompressedImage JPEG → numpy BGR and store it."""
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn(
                    "⚠️ cv2.imdecode returned None — corrupt JPEG frame"
                )
                return
            self._latest_frame = frame
        except Exception as exc:
            self.get_logger().error(f"❌ Failed to decode camera frame: {exc}")

    # ── 10 Hz inference timer ──────────────────────────────────────────

    def _inference_timer_callback(self):
        """Run inference on the latest frame and publish results as JSON."""
        # Early return — nothing to do
        if self._latest_frame is None:
            return
        if self._active_engine is None:
            return

        try:
            # ── YOLO-based engines (yolo / face / pose) ────────────────
            if self._active_engine in ("yolo", "face", "pose"):
                if self._model is None:
                    return
                results = self._model(self._latest_frame, verbose=False)
                if self._active_engine == "yolo":
                    detections = self._build_detections(results)
                    det_msg = String()
                    det_msg.data = json.dumps(detections)
                    self._detections_pub.publish(det_msg)
                elif self._active_engine == "face":
                    faces = self._build_faces(results)
                    det_msg = String()
                    det_msg.data = json.dumps(faces)
                    self._faces_pub.publish(det_msg)
                elif self._active_engine == "pose":
                    pose = self._build_pose_keypoints(results)
                    det_msg = String()
                    det_msg.data = json.dumps(pose)
                    self._pose_pub.publish(det_msg)

            # ── ArUco engine ───────────────────────────────────────────
            elif self._active_engine == "aruco":
                if self._aruco_detector is None:
                    return
                self._run_aruco_detection()

            # ── Gesture engine ─────────────────────────────────────────
            elif self._active_engine == "gesture":
                if self._gesture_recognizer is None:
                    return
                self._run_gesture_recognition()

        except Exception as exc:
            self.get_logger().error(f"❌ Inference error: {exc}")

    # ── Model loading (lazy ultralytics import) ────────────────────────

    def _load_model(self, model_name: str):
        """Load a YOLO model from ultralytics.

        ``ultralytics`` is imported lazily inside this method so the node
        can start without it installed.  A clear error message is logged
        if the import fails.
        """
        self.get_logger().info(f"📦 Loading YOLO model: {model_name} ...")
        t0 = time.time()

        try:
            from ultralytics import YOLO  # noqa: delayed import
        except ImportError:
            self.get_logger().fatal(
                "❌ ultralytics is not installed!  run: pip install ultralytics"
            )
            self._model = None
            self._active_model_name = None
            self._active_engine = None
            # Let bridge know we failed
            mode_msg = String()
            mode_msg.data = "disable"
            self._mode_pub.publish(mode_msg)
            return

        try:
            import os
            import subprocess
            
            # Store models cleanly inside bonicbot_bridge/models/
            models_dir = os.path.join(os.path.dirname(__file__), 'models')
            os.makedirs(models_dir, exist_ok=True)
            model_path = os.path.join(models_dir, model_name)
            
            if model_name == 'yolov8n-face.pt' and (
                not os.path.exists(model_path) or os.path.getsize(model_path) < 1_000_000
            ):
                self.get_logger().info(f"⬇️ Auto-downloading custom face weights: {model_name}...")
                url = f"https://github.com/akanametov/yolo-face/releases/download/1.0.0/{model_name}"
                subprocess.check_call(['wget', '-q', '-O', model_path, url])

            self._model = YOLO(model_path)
            self._active_model_name = model_name
            elapsed = time.time() - t0
            self.get_logger().info(
                f"✅ YOLO model loaded: {model_name} in {elapsed:.2f}s"
            )
        except Exception as exc:
            self.get_logger().error(
                f"❌ Failed to load YOLO model '{model_name}': {exc}"
            )
            self._model = None
            self._active_model_name = None
            self._active_engine = None
            # Publish failure to bridge
            mode_msg = String()
            mode_msg.data = "disable"
            self._mode_pub.publish(mode_msg)

    # ── Results → JSON conversion ──────────────────────────────────────

    def _build_detections(self, results) -> list:
        """Convert ultralytics Results → list[dict] matching the bridge schema.

        Filters detections below ``CONFIDENCE_THRESHOLD``.
        All values are explicitly cast from numpy types to Python native
        types so ``json.dumps()`` does not raise ``TypeError``.
        """
        detections = []

        if not results or len(results) == 0:
            return detections

        result = results[0]
        boxes = result.boxes
        names = result.names  # {int: str} class-id → class-name mapping

        if boxes is None or len(boxes) == 0:
            return detections

        for box in boxes:
            conf = float(box.conf[0])
            if conf < CONFIDENCE_THRESHOLD:
                continue

            # xyxy → x, y, w, h (top-left origin)
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x = int(x1)
            y = int(y1)
            w = int(x2 - x1)
            h = int(y2 - y1)

            cls_id = int(box.cls[0])
            cls_name = names.get(cls_id, f"class_{cls_id}")

            center_x = round(float(x1 + x2) / 2.0, 1)
            center_y = round(float(y1 + y2) / 2.0, 1)

            detections.append(
                {
                    "class": cls_name,
                    "confidence": round(conf, 3),
                    "bbox": [x, y, w, h],
                    "center_x": center_x,
                    "center_y": center_y,
                }
            )

        return detections

    def _build_faces(self, results) -> list:
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

    def _build_pose_keypoints(self, results) -> dict:
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

    def _load_aruco_detector(self, dict_name: str):
        """Create an ArUco detector for the given dictionary name."""
        self.get_logger().info(f"📦 Loading ArUco detector: {dict_name} ...")
        if dict_name not in ARUCO_DICT_MAP:
            self.get_logger().error(
                f"❌ Unknown ArUco dictionary '{dict_name}'. "
                f"Valid: {list(ARUCO_DICT_MAP)}"
            )
            self._aruco_detector = None
            self._aruco_dict_name = None
            self._active_engine = None
            mode_msg = String()
            mode_msg.data = "disable"
            self._mode_pub.publish(mode_msg)
            return

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_MAP[dict_name])
        aruco_params = cv2.aruco.DetectorParameters()

        # ── Tune detection parameters for real-world conditions ─────────
        # Default params are too strict for variable lighting, webcam blur,
        # and JPEG compression artifacts.  These relaxations dramatically
        # improve recall without meaningfully increasing false positives.
        aruco_params.adaptiveThreshWinSizeMin  = 3    # default 3  (keep)
        aruco_params.adaptiveThreshWinSizeMax  = 23   # default 23 → widen
        aruco_params.adaptiveThreshWinSizeStep = 4    # default 10 → finer steps = more chances
        aruco_params.adaptiveThreshConstant    = 7    # default 7  (keep)
        aruco_params.minMarkerPerimeterRate    = 0.02 # default 0.03 → allow smaller markers
        aruco_params.maxMarkerPerimeterRate    = 4.0  # default 4.0 (keep)
        aruco_params.polygonalApproxAccuracyRate = 0.05  # default 0.05 (keep)
        aruco_params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
        aruco_params.errorCorrectionRate       = 0.6  # default 0.6 → allows 60% bit errors

        self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        self._aruco_dict_name = dict_name
        self.get_logger().info(f"✅ ArUco detector loaded: {dict_name} (tuned params)")

    def _run_aruco_detection(self):
        """Run ArUco marker detection on the latest frame."""
        try:
            frame = self._latest_frame
            h, w = frame.shape[:2]

            # ── Adapt camera intrinsics to actual frame size ────────────
            # The default matrix assumes 640x480; if the webcam delivers a
            # different resolution the principal point is wrong, skewing
            # solvePnP and degrading corner detection at frame edges.
            if w != 640 or h != 480:
                scale_x = w / 640.0
                scale_y = h / 480.0
                cam_mtx = self._camera_matrix.copy()
                cam_mtx[0, 0] *= scale_x  # fx
                cam_mtx[1, 1] *= scale_y  # fy
                cam_mtx[0, 2] = w / 2.0   # cx
                cam_mtx[1, 2] = h / 2.0   # cy
            else:
                cam_mtx = self._camera_matrix

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._aruco_detector.detectMarkers(gray)
            markers = self._build_aruco_markers(corners, ids, cam_mtx)
            msg = String()
            msg.data = json.dumps(markers)
            self._aruco_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"❌ ArUco detection error: {exc}")
            msg = String()
            msg.data = "[]"
            self._aruco_pub.publish(msg)

    def _build_aruco_markers(self, corners, ids, camera_matrix=None) -> list:
        """Convert ArUco detection output to list of dicts with pose."""
        markers = []
        if ids is None:
            return markers

        # 3D object points for a single marker (in marker coords)
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

            # Pose estimation via solvePnP
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
        self.get_logger().info("📦 Loading Gesture Recognizer ...")
        try:
            import mediapipe as mp  # noqa: delayed import
        except ImportError:
            self.get_logger().fatal(
                "❌ mediapipe is not installed! run: pip install mediapipe"
            )
            self._gesture_recognizer = None
            self._active_engine = None
            mode_msg = String()
            mode_msg.data = "disable"
            self._mode_pub.publish(mode_msg)
            return

        try:
            import os
            import subprocess
            if not os.path.exists(GESTURE_MODEL_PATH):
                self.get_logger().info(
                    f"⬇️ Auto-downloading gesture model: {GESTURE_MODEL_PATH} ..."
                )
                subprocess.check_call(['wget', '-q', '-O', GESTURE_MODEL_PATH, GESTURE_MODEL_URL])

            base_options = mp.tasks.BaseOptions(
                model_asset_path=GESTURE_MODEL_PATH
            )
            options = mp.tasks.vision.GestureRecognizerOptions(
                base_options=base_options,
                running_mode=mp.tasks.vision.RunningMode.IMAGE,
                num_hands=2,
            )
            self._gesture_recognizer = (
                mp.tasks.vision.GestureRecognizer.create_from_options(options)
            )
            self.get_logger().info("✅ Gesture Recognizer loaded")
        except Exception as exc:
            self.get_logger().error(
                f"❌ Failed to load Gesture Recognizer: {exc}"
            )
            self._gesture_recognizer = None
            self._active_engine = None
            mode_msg = String()
            mode_msg.data = "disable"
            self._mode_pub.publish(mode_msg)

    def _run_gesture_recognition(self):
        """Run MediaPipe gesture recognition on the latest frame."""
        try:
            import mediapipe as mp

            rgb = cv2.cvtColor(self._latest_frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB, data=rgb
            )
            result = self._gesture_recognizer.recognize(mp_image)
            gesture_data = self._build_gesture_result(
                result, self._latest_frame.shape
            )
            msg = String()
            msg.data = json.dumps(gesture_data)
            self._gesture_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"❌ Gesture recognition error: {exc}")
            msg = String()
            msg.data = "{}"
            self._gesture_pub.publish(msg)

    def _build_gesture_result(self, result, frame_shape) -> dict:
        """Convert MediaPipe gesture result to dict."""
        if not result.gestures:
            return {}

        frame_h, frame_w = frame_shape[:2]

        # Take first detected hand
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


# ── Entry point ─────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Vision pipeline shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
