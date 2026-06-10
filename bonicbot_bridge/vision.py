"""Vision controller for robot detection mode management."""
import enum
import json
import time
from roslibpy import Topic
from .exceptions import BonicBotError
from .utils import (
    STRING_MESSAGE_TYPE,
    BOOL_MESSAGE_TYPE,
    VISION_CONTROL_TOPIC,
    VISION_DETECTIONS_TOPIC,
    VISION_FACES_TOPIC,
    VISION_POSE_TOPIC,
    VISION_GESTURE_TOPIC,
    VISION_ARUCO_TOPIC,
    VISION_YOLO_ACTIVE_TOPIC,
    VISION_POSE_ACTIVE_TOPIC,
    VISION_FACE_ACTIVE_TOPIC,
    VISION_GESTURE_ACTIVE_TOPIC,
    VISION_ARUCO_ACTIVE_TOPIC,
    safe_unsubscribe,
    safe_unadvertise,
)

class VisionError(BonicBotError):
    """Raised on general vision control failures (publish error, not connected)."""
    pass

class DetectionModeError(VisionError):
    """Raised when an invalid DetectionMode is passed to enable_detection()."""
    pass

class DetectionMode(enum.Enum):
    """Valid detection pipeline modes."""
    FACE = 'face'
    POSE = 'pose'
    OBJECT = 'object'
    LINE = 'line'
    YOLO = 'yolo'
    GESTURE = 'gesture'
    ARUCO = 'aruco'
    NONE = 'disable'

DEFAULT_DETECTION_THROTTLE_MS = 100
DEFAULT_DETECTION_TIMEOUT_S = 5.0

class VisionController:
    """Standalone vision-pipeline controller — same pattern as CameraManager."""

    def __init__(self, ros_client):
        """Initialize vision controller."""
        self.ros = ros_client
        self._shutdown_called = False

        # Active detector states (updated by individual Bool subscribers)
        self._yolo_active = False
        self._pose_active = False
        self._face_active = False
        self._gesture_active = False
        self._aruco_active = False

        # Publisher for /vision/control
        self._vision_pub = Topic(self.ros, VISION_CONTROL_TOPIC, STRING_MESSAGE_TYPE)
        self._vision_pub.advertise()

        # Individual active-status subscribers (Bool topics from vision_pipeline.py)
        self._yolo_active_sub = Topic(self.ros, VISION_YOLO_ACTIVE_TOPIC, BOOL_MESSAGE_TYPE)
        self._yolo_active_sub.subscribe(lambda msg: self._on_active('yolo', msg))

        self._pose_active_sub = Topic(self.ros, VISION_POSE_ACTIVE_TOPIC, BOOL_MESSAGE_TYPE)
        self._pose_active_sub.subscribe(lambda msg: self._on_active('pose', msg))

        self._face_active_sub = Topic(self.ros, VISION_FACE_ACTIVE_TOPIC, BOOL_MESSAGE_TYPE)
        self._face_active_sub.subscribe(lambda msg: self._on_active('face', msg))

        self._gesture_active_sub = Topic(self.ros, VISION_GESTURE_ACTIVE_TOPIC, BOOL_MESSAGE_TYPE)
        self._gesture_active_sub.subscribe(lambda msg: self._on_active('gesture', msg))

        self._aruco_active_sub = Topic(self.ros, VISION_ARUCO_ACTIVE_TOPIC, BOOL_MESSAGE_TYPE)
        self._aruco_active_sub.subscribe(lambda msg: self._on_active('aruco', msg))

        # Detection result subscribers
        self._latest_detections: list[dict] = []
        self._detections_sub = Topic(self.ros, VISION_DETECTIONS_TOPIC, STRING_MESSAGE_TYPE, throttle_rate=DEFAULT_DETECTION_THROTTLE_MS)
        self._detections_sub.subscribe(self._on_detections)

        self._latest_faces: list[dict] = []
        self._faces_sub = Topic(self.ros, VISION_FACES_TOPIC, STRING_MESSAGE_TYPE, throttle_rate=DEFAULT_DETECTION_THROTTLE_MS)
        self._faces_sub.subscribe(self._on_faces)

        self._latest_pose: dict = {}
        self._pose_sub = Topic(self.ros, VISION_POSE_TOPIC, STRING_MESSAGE_TYPE, throttle_rate=DEFAULT_DETECTION_THROTTLE_MS)
        self._pose_sub.subscribe(self._on_pose)

        self._latest_gesture: dict = {}
        self._gesture_sub = Topic(self.ros, VISION_GESTURE_TOPIC, STRING_MESSAGE_TYPE, throttle_rate=DEFAULT_DETECTION_THROTTLE_MS)
        self._gesture_sub.subscribe(self._on_gesture)

        self._latest_aruco: list[dict] = []
        self._aruco_sub = Topic(self.ros, VISION_ARUCO_TOPIC, STRING_MESSAGE_TYPE, throttle_rate=0)
        self._aruco_sub.subscribe(self._on_aruco)

    # ── Active-status callback ──────────────────────────────────────────

    def _on_active(self, detector, msg):
        """Update individual detector active flag from Bool subscription."""
        value = msg.get('data', False)
        if detector == 'yolo':
            self._yolo_active = value
        elif detector == 'pose':
            self._pose_active = value
        elif detector == 'face':
            self._face_active = value
        elif detector == 'gesture':
            self._gesture_active = value
        elif detector == 'aruco':
            self._aruco_active = value

    # ── Detection result callbacks ──────────────────────────────────────

    def _on_detections(self, msg):
        """Update latest detections from /vision/yolo_detections."""
        try:
            self._latest_detections = json.loads(msg['data'])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f'⚠️ vision: failed to parse detections: {exc}')
            self._latest_detections = []

    def _on_faces(self, msg):
        """Update latest faces from /vision/face_detections."""
        try:
            self._latest_faces = json.loads(msg['data'])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f'⚠️ vision: failed to parse faces: {exc}')
            self._latest_faces = []

    def _on_pose(self, msg):
        """Update latest pose from /vision/pose_landmarks."""
        try:
            self._latest_pose = json.loads(msg['data'])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f'⚠️ vision: failed to parse pose: {exc}')
            self._latest_pose = {}

    def _on_gesture(self, msg):
        """Update latest gesture from /vision/gestures."""
        try:
            self._latest_gesture = json.loads(msg['data'])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f'⚠️ vision: failed to parse gesture: {exc}')
            self._latest_gesture = {}

    def _on_aruco(self, msg):
        """Update latest ArUco markers from /vision/aruco_ids."""
        try:
            self._latest_aruco = json.loads(msg['data'])
        except (json.JSONDecodeError, KeyError) as exc:
            print(f'⚠️ vision: failed to parse aruco: {exc}')
            self._latest_aruco = []

    # ── Public API ──────────────────────────────────────────────────────

    def enable_detection(self, mode, model='yolov8n', dictionary=None):
        """Enable a detection pipeline mode."""
        if not isinstance(mode, DetectionMode):
            valid = [m.name for m in DetectionMode if m is not DetectionMode.NONE]
            raise DetectionModeError(f'mode must be a DetectionMode enum member, got {type(mode).__name__}. Valid modes: {valid}')
        if mode is DetectionMode.NONE:
            raise DetectionModeError('use disable_detection() to stop the vision pipeline')
        if mode is DetectionMode.LINE:
            raise DetectionModeError('LINE detection mode is not yet implemented')
        config = {
            "yolo": False,
            "pose": False,
            "face": False,
            "gesture": False,
            "aruco": False
        }

        if mode is DetectionMode.YOLO or mode is DetectionMode.OBJECT:
            config["yolo"] = True
        elif mode is DetectionMode.POSE:
            config["pose"] = True
        elif mode is DetectionMode.FACE:
            config["face"] = True
        elif mode is DetectionMode.GESTURE:
            config["gesture"] = True
        elif mode is DetectionMode.ARUCO:
            config["aruco"] = True

        payload = json.dumps(config)
        try:
            self._vision_pub.publish({'data': payload})
        except Exception as exc:
            print(f'⚠️ Failed to publish enable_detection({mode.name}): {exc}')
            raise VisionError(f"Failed to enable detection mode '{mode.name}': {exc}")
        print(f'👁️ Vision detection enabled: {mode.name}')
        return True

    def disable_detection(self):
        """Disable the detection pipeline."""
        config = {
            "yolo": False,
            "pose": False,
            "face": False,
            "gesture": False,
            "aruco": False
        }
        try:
            self._vision_pub.publish({'data': json.dumps(config)})
            print('🛑 Vision detection disabled')
            return True
        except Exception as exc:
            print(f'⚠️ Failed to publish disable_detection: {exc}')
            raise VisionError(f'Failed to disable detection: {exc}')

    def get_active_mode(self):
        """Return a dict of active detector states."""
        return {
            'yolo': self._yolo_active,
            'pose': self._pose_active,
            'face': self._face_active,
            'gesture': self._gesture_active,
            'aruco': self._aruco_active,
        }

    def is_detector_active(self, detector_name):
        """Check if a specific detector is active on the robot.

        Args:
            detector_name: 'yolo', 'pose', 'face', 'gesture', or 'aruco'.

        Returns:
            bool: True if the detector is currently running on the robot.
        """
        return self.get_active_mode().get(detector_name, False)

    def get_detections(self, class_filter=None):
        """Return the latest detection results from the vision pipeline."""
        if class_filter is None:
            return list(self._latest_detections)
        return [d for d in self._latest_detections if d.get('class') == class_filter]

    def get_faces(self) -> list[dict]:
        """Return the latest face detections."""
        return list(self._latest_faces)

    def get_pose_keypoints(self) -> dict:
        """Return the latest pose keypoints."""
        return dict(self._latest_pose)

    def get_gesture(self) -> 'str | None':
        """Return the current gesture class name, or None if no hand detected."""
        return self._latest_gesture.get('gesture')

    def get_gesture_full(self) -> dict:
        """Return the full gesture result including landmarks."""
        return dict(self._latest_gesture)

    def get_aruco_markers(self) -> list:
        """Return the latest ArUco marker detections."""
        return list(self._latest_aruco)

    def wait_for_marker(self, marker_id: int, timeout: float=5.0):
        """Block until a specific ArUco marker ID is detected, or timeout."""
        start = time.time()
        while time.time() - start < timeout:
            for m in self.get_aruco_markers():
                if m.get('id') == marker_id:
                    return m
            time.sleep(0.05)
        print(f'⏰ wait_for_marker: ID {marker_id} not found after {timeout}s')
        return None

    def wait_for_detection(self, target_class, timeout=DEFAULT_DETECTION_TIMEOUT_S):
        """Block until a detection of ``target_class`` appears, or timeout."""
        start = time.time()
        while time.time() - start < timeout:
            matches = self.get_detections(class_filter=target_class)
            if matches:
                return matches[0]
            time.sleep(0.05)
        print(f"⏰ wait_for_detection: '{target_class}' not found after {timeout}s")
        return None

    def wait_for_gesture(self, gesture_name: str, timeout: float=5.0):
        """Block until a specific gesture is detected, or timeout."""
        start = time.time()
        while time.time() - start < timeout:
            current = self.get_gesture()
            if current and current.lower() == gesture_name.lower():
                return self.get_gesture_full()
            time.sleep(0.05)
        print(f"⏰ wait_for_gesture: '{gesture_name}' not detected after {timeout}s")
        return None

    def wait_for_face(self, timeout: float=5.0):
        """Block until any face is detected, or timeout."""
        start = time.time()
        while time.time() - start < timeout:
            faces = self.get_faces()
            if faces:
                return faces[0]
            time.sleep(0.05)
        print(f'⏰ wait_for_face: no face detected after {timeout}s')
        return None

    def wait_for_pose(self, timeout: float=5.0):
        """Block until pose keypoints are detected, or timeout."""
        start = time.time()
        while time.time() - start < timeout:
            pose = self.get_pose_keypoints()
            if pose:
                return pose
            time.sleep(0.05)
        print(f'⏰ wait_for_pose: no pose detected after {timeout}s')
        return None

    # ── Teardown ────────────────────────────────────────────────────────

    def shutdown(self):
        """Release vision subscriptions and publisher."""
        if self._shutdown_called:
            return
        self._shutdown_called = True
        safe_unsubscribe(self._yolo_active_sub)
        safe_unsubscribe(self._pose_active_sub)
        safe_unsubscribe(self._face_active_sub)
        safe_unsubscribe(self._gesture_active_sub)
        safe_unsubscribe(self._aruco_active_sub)
        safe_unsubscribe(self._detections_sub)
        safe_unsubscribe(self._faces_sub)
        safe_unsubscribe(self._pose_sub)
        safe_unsubscribe(self._gesture_sub)
        safe_unsubscribe(self._aruco_sub)
        safe_unadvertise(self._vision_pub)