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

import enum
import json
import time

from roslibpy import Topic

from .exceptions import BonicBotError
from .utils import safe_unsubscribe, safe_unadvertise

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


# ── Module-level topic / message constants ─────────────────────────────

VISION_CONTROL_TOPIC = "/vision/control"
VISION_MODE_TOPIC = "/robot/vision_mode"
VISION_DETECTIONS_TOPIC = "/vision/detections"
VISION_FACES_TOPIC = "/vision/faces"
VISION_POSE_TOPIC = "/vision/pose"
VISION_GESTURE_TOPIC = "/vision/gesture"
VISION_ARUCO_TOPIC = "/vision/aruco"
VISION_MESSAGE_TYPE = "std_msgs/String"
DEFAULT_DETECTION_THROTTLE_MS = 100  # 10 Hz over WebSocket
DEFAULT_DETECTION_TIMEOUT_S = 5.0


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
            VISION_MESSAGE_TYPE,
        )
        self._vision_pub.advertise()

        # ── Live mode subscriber (background — stays alive for session) ─
        self._vision_mode_sub = Topic(
            self.ros,
            VISION_MODE_TOPIC,
            VISION_MESSAGE_TYPE,
        )
        self._vision_mode_sub.subscribe(self._on_vision_mode)

        # ── Detections subscriber (throttled — stays alive for session) ──
        self._latest_detections: list[dict] = []
        self._detections_sub = Topic(
            self.ros,
            VISION_DETECTIONS_TOPIC,
            VISION_MESSAGE_TYPE,
            throttle_rate=DEFAULT_DETECTION_THROTTLE_MS,
        )
        self._detections_sub.subscribe(self._on_detections)

        # ── Faces subscriber ─────────────────────────────────────────────
        self._latest_faces: list[dict] = []
        self._faces_sub = Topic(
            self.ros,
            VISION_FACES_TOPIC,
            VISION_MESSAGE_TYPE,
            throttle_rate=DEFAULT_DETECTION_THROTTLE_MS,
        )
        self._faces_sub.subscribe(self._on_faces)

        # ── Pose subscriber ──────────────────────────────────────────────
        self._latest_pose: dict = {}
        self._pose_sub = Topic(
            self.ros,
            VISION_POSE_TOPIC,
            VISION_MESSAGE_TYPE,
            throttle_rate=DEFAULT_DETECTION_THROTTLE_MS,
        )
        self._pose_sub.subscribe(self._on_pose)

        # ── Gesture subscriber ─────────────────────────────────────────────
        self._latest_gesture: dict = {}
        self._gesture_sub = Topic(
            self.ros,
            VISION_GESTURE_TOPIC,
            VISION_MESSAGE_TYPE,
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
            VISION_MESSAGE_TYPE,
            throttle_rate=0,
        )
        self._aruco_sub.subscribe(self._on_aruco)

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

    def enable_detection(self, mode, model="yolov8n", dictionary=None):
        """Enable a detection pipeline mode.

        Publishes the mode value to /vision/control via the shared
        publisher that was advertised in __init__.

        For YOLO mode, the payload is ``'yolo:<model>'`` (colon-separated).
        For ARUCO mode, an optional ``dictionary`` parameter can be passed
        to select the ArUco dictionary (default: DICT_4X4_50).

        Args:
            mode (DetectionMode): The detection mode to enable.
            model (str): YOLO model name (default: 'yolov8n').
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

        # Build payload
        if mode is DetectionMode.YOLO:
            payload = f"yolo:{model}"
        elif mode is DetectionMode.ARUCO and dictionary is not None:
            payload = f"aruco:{dictionary}"
        else:
            payload = mode.value

        # Publish
        try:
            self._vision_pub.publish({"data": payload})
            print(f"👁️ Vision detection enabled: {mode.name} ({payload})")
            return True
        except Exception as exc:
            print(f"⚠️ Failed to publish enable_detection({mode.name}): {exc}")
            raise VisionError(f"Failed to enable detection mode '{payload}': {exc}")

    def disable_detection(self):
        """Disable the detection pipeline.

        Publishes DetectionMode.NONE.value ('disable') to
        /vision/control via the shared publisher.

        Returns:
            bool: True on successful publish.

        Raises:
            VisionError: If the publish call fails.
        """
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

    # ── Teardown ───────────────────────────────────────────────────────

    def shutdown(self):
        """Release vision subscriptions and publisher.

        Safe to call multiple times — the ``_shutdown_called`` guard
        ensures idempotency (same pattern as CameraManager.shutdown).
        """
        if self._shutdown_called:
            return

        self._shutdown_called = True

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
