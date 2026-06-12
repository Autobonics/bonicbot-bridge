"""
VisionController — Python SDK wrapper for the BonicBot vision pipeline.

Mirrors the vision integration from ros_bridge_service.dart.

Architecture
────────────
The vision pipeline runs on the robot (RPi 4) as a ROS 2 node (vision_pipeline.py).
This class talks to it through rosbridge (roslibpy).

Detectors
─────────
  yolo     — YOLO object detection  → /vision/yolo_detections   (JSON String)
  pose     — MediaPipe body pose    → /vision/pose_landmarks     (JSON String)
  face     — MediaPipe face detect  → /vision/face_detections    (JSON String)
  gesture  — MediaPipe hand gesture → /vision/gestures           (JSON String)
  aruco    — ArUco marker detect    → /vision/aruco_ids          (JSON String)

Control
───────
Publish a JSON command to /vision/control  (latched, std_msgs/String):
  {"yolo": true, "pose": false, "face": true, "gesture": false, "aruco": false}

The pipeline services on the robot are called for start/stop/enable/disable:
  /robot/start_vision   /robot/stop_vision
  /robot/enable_yolo    /robot/disable_yolo
  /robot/enable_face    /robot/disable_face
  /robot/enable_pose    /robot/disable_pose
  /robot/enable_gesture /robot/disable_gesture
  /robot/enable_aruco   /robot/disable_aruco

Lazy subscription
─────────────────
Data topics are *not* subscribed at init time (they can be bandwidth-heavy).
Call subscribe_to_vision_pipeline() to start receiving data, or pass
auto_subscribe=True to __init__ (default False).
Data topic subscriptions are torn down by unsubscribe_from_vision_pipeline().
"""

import json
import threading
import time

from roslibpy import Service, ServiceRequest, Topic

from .utils import (
    # message types
    BOOL_MESSAGE_TYPE,
    STRING_MESSAGE_TYPE,
    TRIGGER_SERVICE_TYPE,
    # vision control
    VISION_CONTROL_TOPIC,
    # vision active-status topics
    VISION_YOLO_ACTIVE_TOPIC,
    VISION_POSE_ACTIVE_TOPIC,
    VISION_FACE_ACTIVE_TOPIC,
    VISION_GESTURE_ACTIVE_TOPIC,
    VISION_ARUCO_ACTIVE_TOPIC,
    # vision data-output topics
    VISION_YOLO_DETECTIONS_TOPIC,
    VISION_POSE_LANDMARKS_TOPIC,
    VISION_FACE_DETECTIONS_TOPIC,
    VISION_GESTURES_TOPIC,
    VISION_ARUCO_IDS_TOPIC,
    VISION_NEAREST_PERSON_TOPIC,
    # robot-level services
    ROBOT_START_VISION_SERVICE,
    ROBOT_STOP_VISION_SERVICE,
    ROBOT_ENABLE_YOLO_SERVICE,
    ROBOT_DISABLE_YOLO_SERVICE,
    ROBOT_ENABLE_FACE_SERVICE,
    ROBOT_DISABLE_FACE_SERVICE,
    ROBOT_ENABLE_POSE_SERVICE,
    ROBOT_DISABLE_POSE_SERVICE,
    ROBOT_ENABLE_GESTURE_SERVICE,
    ROBOT_DISABLE_GESTURE_SERVICE,
    ROBOT_ENABLE_ARUCO_SERVICE,
    ROBOT_DISABLE_ARUCO_SERVICE,
    # helpers
    safe_unsubscribe,
    safe_unadvertise,
    call_trigger_service,
)
from .exceptions import BonicBotError

# ── Tunable defaults ──────────────────────────────────────────────────────────
DEFAULT_SERVICE_TIMEOUT_SECONDS = 10.0
DEFAULT_WAIT_POLL_SECONDS       = 0.05
# ─────────────────────────────────────────────────────────────────────────────

# All detector keys used in /vision/control JSON and throughout this class.
_ALL_DETECTORS = ("yolo", "pose", "face", "gesture", "aruco")


class VisionController:
    

    def __init__(self, ros_client, auto_subscribe: bool = False):
        
        self._ros = ros_client

        # ── Internal state ────────────────────────────────────────────────────
        self._vision_active   = False
        self._yolo_active     = False
        self._pose_active     = False
        self._face_active     = False
        self._gesture_active  = False
        self._aruco_active    = False

        # Latest detection payloads (populated by subscription callbacks)
        self._yolo_detections: list  = []
        self._pose_landmarks:  list  = []
        self._face_detections: list  = []
        self._gestures:        list  = []
        self._aruco_ids:       list  = []
        self._nearest_person:  dict | None = None

        # ── Subscription tracking (lazy) ──────────────────────────────────────
        self._is_vision_subscribed = False
        self._is_vision_wanted     = False

        # ── Per-topic handles (populated by _setup_status_topics / subscribe_*) ─
        # Status (active-flag) subscribers — set up eagerly so we always know
        # which detectors are on, even before the data subscription is opened.
        self._yolo_active_sub    = None
        self._pose_active_sub    = None
        self._face_active_sub    = None
        self._gesture_active_sub = None
        self._aruco_active_sub   = None

        # Data subscribers — created lazily
        self._yolo_det_sub     = None
        self._pose_land_sub    = None
        self._face_det_sub     = None
        self._gesture_sub      = None
        self._aruco_ids_sub    = None
        self._nearest_per_sub  = None

        # Control publisher (latched — mirrors Dart latch: true)
        self._vision_ctrl_pub = Topic(
            self._ros,
            VISION_CONTROL_TOPIC,
            STRING_MESSAGE_TYPE,
            latch=True,
        )
        self._vision_ctrl_pub.advertise()

        # ── Thread safety ─────────────────────────────────────────────────────
        self._lock = threading.Lock()

        # Eagerly subscribe to active-status topics so state is always accurate.
        self._setup_status_topics()

        if auto_subscribe:
            self.subscribe_to_vision_pipeline()

    # =========================================================================
    # Internal — status topic setup
    # =========================================================================

    def _setup_status_topics(self):
        
        _status_map = [
            (VISION_YOLO_ACTIVE_TOPIC,    self._on_yolo_active),
            (VISION_POSE_ACTIVE_TOPIC,    self._on_pose_active),
            (VISION_FACE_ACTIVE_TOPIC,    self._on_face_active),
            (VISION_GESTURE_ACTIVE_TOPIC, self._on_gesture_active),
            (VISION_ARUCO_ACTIVE_TOPIC,   self._on_aruco_active),
        ]
        for topic_name, callback in _status_map:
            try:
                sub = Topic(self._ros, topic_name, BOOL_MESSAGE_TYPE)
                sub.subscribe(callback)
                # Store handle for cleanup
                attr = f"_{topic_name.split('/')[-1].replace('_active', '')}_active_sub"
                # e.g.  /vision/yolo_active  →  _yolo_active_sub
                setattr(self, attr, sub)
            except Exception as exc:
                print(f"⚠️ VisionController: could not subscribe to {topic_name}: {exc}")

    # ── Status callbacks ──────────────────────────────────────────────────────

    def _on_yolo_active(self, msg):
        with self._lock:
            self._yolo_active = bool(msg.get("data", False))

    def _on_pose_active(self, msg):
        with self._lock:
            self._pose_active = bool(msg.get("data", False))

    def _on_face_active(self, msg):
        with self._lock:
            self._face_active = bool(msg.get("data", False))

    def _on_gesture_active(self, msg):
        with self._lock:
            self._gesture_active = bool(msg.get("data", False))

    def _on_aruco_active(self, msg):
        with self._lock:
            self._aruco_active = bool(msg.get("data", False))

    # =========================================================================
    # Internal — data topic callbacks
    # =========================================================================

    def _on_yolo_detections(self, msg):
        try:
            raw = msg.get("data", "[]")
            decoded = json.loads(raw)
            with self._lock:
                self._yolo_detections = decoded if isinstance(decoded, list) else []
        except Exception as exc:
            print(f"⚠️ VisionController: error parsing YOLO detections: {exc}")

    def _on_pose_landmarks(self, msg):
        try:
            raw     = msg.get("data", "{}")
            decoded = json.loads(raw)
            with self._lock:
                if isinstance(decoded, dict) and "pose_landmarks" in decoded:
                    self._pose_landmarks = decoded["pose_landmarks"]
                elif isinstance(decoded, list):
                    self._pose_landmarks = decoded
                else:
                    self._pose_landmarks = []
        except Exception as exc:
            print(f"⚠️ VisionController: error parsing pose landmarks: {exc}")

    def _on_face_detections(self, msg):
        try:
            raw     = msg.get("data", "[]")
            decoded = json.loads(raw)
            with self._lock:
                self._face_detections = decoded if isinstance(decoded, list) else []
        except Exception as exc:
            print(f"⚠️ VisionController: error parsing face detections: {exc}")

    def _on_gestures(self, msg):
        try:
            raw     = msg.get("data", "[]")
            decoded = json.loads(raw)
            with self._lock:
                self._gestures = decoded if isinstance(decoded, list) else []
        except Exception as exc:
            print(f"⚠️ VisionController: error parsing gestures: {exc}")

    def _on_aruco_ids(self, msg):
        try:
            raw     = msg.get("data", "[]")
            decoded = json.loads(raw)
            with self._lock:
                if isinstance(decoded, list):
                    self._aruco_ids = [int(x) for x in decoded]
                else:
                    self._aruco_ids = []
        except Exception as exc:
            print(f"⚠️ VisionController: error parsing ArUco IDs: {exc}")

    def _on_nearest_person(self, msg):
        try:
            raw = msg.get("data", "null")
            with self._lock:
                self._nearest_person = json.loads(raw)
        except Exception as exc:
            print(f"⚠️ VisionController: error parsing nearest person: {exc}")

    # =========================================================================
    # Internal — service helper
    # =========================================================================

    def _call_trigger(self, service_name: str, timeout: float = DEFAULT_SERVICE_TIMEOUT_SECONDS) -> dict:
        """Call a std_srvs/Trigger service and return the response dict."""
        svc = Service(self._ros, service_name, TRIGGER_SERVICE_TYPE)
        return call_trigger_service(
            service=svc,
            timeout_secs=timeout,
            exception_cls=BonicBotError,
            error_prefix=f"Vision service [{service_name}]",
        )

    # =========================================================================
    # Internal — /vision/control publisher
    # =========================================================================

    def _publish_control(self, **detector_flags):
        """
        Publish a JSON control command to /vision/control.

        Keyword args are detector names (yolo/pose/face/gesture/aruco) mapped
        to bool.  Unspecified detectors keep their current state.

        Example::
            self._publish_control(yolo=True, face=True)
        """
        payload = {
            "yolo":    self._yolo_active,
            "pose":    self._pose_active,
            "face":    self._face_active,
            "gesture": self._gesture_active,
            "aruco":   self._aruco_active,
        }
        payload.update({k: bool(v) for k, v in detector_flags.items()
                        if k in _ALL_DETECTORS})

        try:
            self._vision_ctrl_pub.publish({"data": json.dumps(payload)})
        except Exception as exc:
            print(f"⚠️ VisionController: failed to publish control: {exc}")

    # =========================================================================
    # Public — lazy subscription management
    # =========================================================================

    def subscribe_to_vision_pipeline(self):
        
        self._is_vision_wanted = True

        if not self._ros.is_connected:
            print("ℹ️ VisionController: not connected — subscription deferred")
            return

        if self._is_vision_subscribed:
            return

        print("📡 VisionController: subscribing to vision pipeline topics…")
        try:
            self._yolo_det_sub = Topic(
                self._ros, VISION_YOLO_DETECTIONS_TOPIC, STRING_MESSAGE_TYPE)
            self._yolo_det_sub.subscribe(self._on_yolo_detections)

            self._pose_land_sub = Topic(
                self._ros, VISION_POSE_LANDMARKS_TOPIC, STRING_MESSAGE_TYPE)
            self._pose_land_sub.subscribe(self._on_pose_landmarks)

            self._face_det_sub = Topic(
                self._ros, VISION_FACE_DETECTIONS_TOPIC, STRING_MESSAGE_TYPE)
            self._face_det_sub.subscribe(self._on_face_detections)

            self._gesture_sub = Topic(
                self._ros, VISION_GESTURES_TOPIC, STRING_MESSAGE_TYPE)
            self._gesture_sub.subscribe(self._on_gestures)

            self._aruco_ids_sub = Topic(
                self._ros, VISION_ARUCO_IDS_TOPIC, STRING_MESSAGE_TYPE)
            self._aruco_ids_sub.subscribe(self._on_aruco_ids)

            self._nearest_per_sub = Topic(
                self._ros, VISION_NEAREST_PERSON_TOPIC, STRING_MESSAGE_TYPE)
            self._nearest_per_sub.subscribe(self._on_nearest_person)

            self._is_vision_subscribed = True
            print("✅ VisionController: vision pipeline subscribed")
        except Exception as exc:
            print(f"❌ VisionController: subscription failed: {exc}")

    def unsubscribe_from_vision_pipeline(self):
        
        self._is_vision_wanted = False

        if not self._is_vision_subscribed:
            return

        print("📡 VisionController: unsubscribing from vision pipeline topics…")
        for attr in (
            "_yolo_det_sub", "_pose_land_sub", "_face_det_sub",
            "_gesture_sub", "_aruco_ids_sub", "_nearest_per_sub",
        ):
            safe_unsubscribe(getattr(self, attr, None))
            setattr(self, attr, None)

        with self._lock:
            self._yolo_detections = []
            self._pose_landmarks  = []
            self._face_detections = []
            self._gestures        = []
            self._aruco_ids       = []
            self._nearest_person  = None

        self._is_vision_subscribed = False
        print("✅ VisionController: vision pipeline unsubscribed")

    # =========================================================================
    # Public — robot-level vision pipeline on/off
    # =========================================================================

    def start_vision(self) -> dict:
       
        result = self._call_trigger(ROBOT_START_VISION_SERVICE)
        with self._lock:
            self._vision_active = True
        if not self._is_vision_subscribed:
            self.subscribe_to_vision_pipeline()
        print("🎥 Vision pipeline started")
        return result

    def stop_vision(self) -> dict:
        
        result = self._call_trigger(ROBOT_STOP_VISION_SERVICE)
        with self._lock:
            self._vision_active  = False
            self._yolo_active    = False
            self._pose_active    = False
            self._face_active    = False
            self._gesture_active = False
            self._aruco_active   = False
        print("🛑 Vision pipeline stopped")
        return result

    # =========================================================================
    # Public — per-detector enable / disable (service calls)
    # =========================================================================

    _DETECTOR_SERVICES = {
        "yolo":    (ROBOT_ENABLE_YOLO_SERVICE,    ROBOT_DISABLE_YOLO_SERVICE),
        "face":    (ROBOT_ENABLE_FACE_SERVICE,     ROBOT_DISABLE_FACE_SERVICE),
        "pose":    (ROBOT_ENABLE_POSE_SERVICE,     ROBOT_DISABLE_POSE_SERVICE),
        "gesture": (ROBOT_ENABLE_GESTURE_SERVICE,  ROBOT_DISABLE_GESTURE_SERVICE),
        "aruco":   (ROBOT_ENABLE_ARUCO_SERVICE,    ROBOT_DISABLE_ARUCO_SERVICE),
    }

    def enable_detector(self, detector: str) -> dict:
       
        detector = detector.lower()
        if detector not in self._DETECTOR_SERVICES:
            raise BonicBotError(
                f"Unknown detector '{detector}'. Valid: {list(self._DETECTOR_SERVICES)}"
            )
        enable_svc, _ = self._DETECTOR_SERVICES[detector]
        result = self._call_trigger(enable_svc)
        # Also mirror state via /vision/control so the pipeline gets the command
        self._publish_control(**{detector: True})
        print(f"✅ Detector enabled: {detector}")
        return result

    def disable_detector(self, detector: str) -> dict:
       
        detector = detector.lower()
        if detector not in self._DETECTOR_SERVICES:
            raise BonicBotError(
                f"Unknown detector '{detector}'. Valid: {list(self._DETECTOR_SERVICES)}"
            )
        _, disable_svc = self._DETECTOR_SERVICES[detector]
        result = self._call_trigger(disable_svc)
        self._publish_control(**{detector: False})
        print(f"🛑 Detector disabled: {detector}")
        return result

    def toggle_detector(self, detector: str, enable: bool) -> dict:
        """Enable or disable a detector in one call.  Mirrors Dart toggleDetector()."""
        if enable:
            return self.enable_detector(detector)
        return self.disable_detector(detector)

    # =========================================================================
    # Public — high-level convenience API (matches core.py delegates)
    # =========================================================================

    def enable_detection(self, mode: str, model: str = "yolov8n", **kwargs) -> dict:
        
        # Ensure pipeline is running first
        if not self._vision_active:
            try:
                self.start_vision()
            except BonicBotError as exc:
                print(f"⚠️ VisionController: start_vision failed: {exc}")

        return self.enable_detector(mode)

    def disable_detection(self) -> dict:
        
        # Disable all active detectors cleanly before stopping the pipeline
        for detector, active_attr in (
            ("yolo",    "_yolo_active"),
            ("pose",    "_pose_active"),
            ("face",    "_face_active"),
            ("gesture", "_gesture_active"),
            ("aruco",   "_aruco_active"),
        ):
            if getattr(self, active_attr, False):
                try:
                    self.disable_detector(detector)
                except BonicBotError:
                    pass

        return self.stop_vision()

    def get_active_mode(self) -> str | None:
        
        with self._lock:
            if self._yolo_active:    return "yolo"
            if self._pose_active:    return "pose"
            if self._face_active:    return "face"
            if self._gesture_active: return "gesture"
            if self._aruco_active:   return "aruco"
        return None

    def get_active_detectors(self) -> list[str]:
        """Return a list of all currently-active detector names."""
        active = []
        with self._lock:
            if self._yolo_active:    active.append("yolo")
            if self._pose_active:    active.append("pose")
            if self._face_active:    active.append("face")
            if self._gesture_active: active.append("gesture")
            if self._aruco_active:   active.append("aruco")
        return active

    # =========================================================================
    # Public — data accessors
    # =========================================================================

    def get_detections(self, class_filter: str | None = None) -> list:
       
        with self._lock:
            detections = list(self._yolo_detections)

        if class_filter:
            cf = class_filter.lower()
            detections = [d for d in detections
                          if d.get("class", "").lower() == cf]
        return detections

    def get_faces(self) -> list:
       
        with self._lock:
            return list(self._face_detections)

    def get_pose_keypoints(self) -> list:
       
        with self._lock:
            return list(self._pose_landmarks)

    def get_gesture(self) -> str | None:
       
        with self._lock:
            if not self._gestures:
                return None
            return self._gestures[0].get("gesture")

    def get_gesture_full(self) -> list:
       
        with self._lock:
            return list(self._gestures)

    def get_aruco_markers(self) -> list[int]:
        """Return the latest list of detected ArUco marker IDs (integers)."""
        with self._lock:
            return list(self._aruco_ids)

    def get_nearest_person(self) -> dict | None:
        
        with self._lock:
            return self._nearest_person

    # ── Status properties ─────────────────────────────────────────────────────

    @property
    def vision_active(self) -> bool:
        """True if the robot-side vision pipeline is running."""
        return self._vision_active

    @property
    def yolo_enabled(self) -> bool:
        with self._lock:
            return self._yolo_active

    @property
    def pose_enabled(self) -> bool:
        with self._lock:
            return self._pose_active

    @property
    def face_enabled(self) -> bool:
        with self._lock:
            return self._face_active

    @property
    def gesture_enabled(self) -> bool:
        with self._lock:
            return self._gesture_active

    @property
    def aruco_enabled(self) -> bool:
        with self._lock:
            return self._aruco_active

    @property
    def is_any_detection_active(self) -> bool:
        """True if at least one detector is currently active."""
        with self._lock:
            return any([
                self._yolo_active,
                self._pose_active,
                self._face_active,
                self._gesture_active,
                self._aruco_active,
            ])

    @property
    def is_subscribed(self) -> bool:
        """True if data topics are currently subscribed."""
        return self._is_vision_subscribed

    # =========================================================================
    # Public — blocking wait helpers (mirrors Dart await pattern)
    # =========================================================================

    def wait_for_detection(self, target_class: str, timeout: float = 5.0) -> list:
       
        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self.get_detections(class_filter=target_class)
            if result:
                return result
            time.sleep(DEFAULT_WAIT_POLL_SECONDS)
        return []

    def wait_for_face(self, timeout: float = 5.0) -> list:
       
        deadline = time.time() + timeout
        while time.time() < deadline:
            faces = self.get_faces()
            if faces:
                return faces
            time.sleep(DEFAULT_WAIT_POLL_SECONDS)
        return []

    def wait_for_pose(self, timeout: float = 5.0) -> list:
        
        deadline = time.time() + timeout
        while time.time() < deadline:
            kps = self.get_pose_keypoints()
            if kps:
                return kps
            time.sleep(DEFAULT_WAIT_POLL_SECONDS)
        return []

    def wait_for_gesture(self, gesture_name: str, timeout: float = 5.0) -> dict | None:
        
        deadline = time.time() + timeout
        while time.time() < deadline:
            for g in self.get_gesture_full():
                if g.get("gesture", "").lower() == gesture_name.lower():
                    return g
            time.sleep(DEFAULT_WAIT_POLL_SECONDS)
        return None

    def wait_for_marker(self, marker_id: int, timeout: float = 5.0) -> bool:
       
        deadline = time.time() + timeout
        while time.time() < deadline:
            if marker_id in self.get_aruco_markers():
                return True
            time.sleep(DEFAULT_WAIT_POLL_SECONDS)
        return False

    # =========================================================================
    # Teardown
    # =========================================================================


    def shutdown(self):
       
        self.unsubscribe_from_vision_pipeline()

        # Unsubscribe status topics
        for attr in (
            "_yolo_active_sub", "_pose_active_sub", "_face_active_sub",
            "_gesture_active_sub", "_aruco_active_sub",
        ):
            safe_unsubscribe(getattr(self, attr, None))
            setattr(self, attr, None)

        safe_unadvertise(self._vision_ctrl_pub)
        self._vision_ctrl_pub = None

        print("🔌 VisionController shutdown complete")