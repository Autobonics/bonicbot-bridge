"""
System controller for high-level robot operations
"""

import time

from roslibpy import Service, ServiceRequest, Topic

from .exceptions import SystemControlError
from .utils import (
    ALREADY_ACTIVE_RESPONSE_TEXT,
    BOOL_MESSAGE_TYPE,
    INACTIVE_RESPONSE_TEXT,
    NAVIGATION_SERVICE_CALL_TIMEOUT_SECONDS,
    START_NAVIGATION_SERVICE,
    STOP_NAVIGATION_SERVICE,
    STRING_MESSAGE_TYPE,
    TRIGGER_SERVICE_TYPE,
    call_trigger_service,
    safe_unsubscribe,
)

START_MAPPING_SERVICE = "/robot/start_mapping"
STOP_MAPPING_SERVICE = "/robot/stop_mapping"
SAVE_MAP_SERVICE = "/robot/save_map"
# START_NAVIGATION_SERVICE and STOP_NAVIGATION_SERVICE are imported from utils
START_CAMERA_SERVICE = "/robot/start_camera"
STOP_CAMERA_SERVICE = "/robot/stop_camera"

ROBOT_STATE_TOPIC = "/robot/state"
MAPPING_STATUS_TOPIC = "/robot/mapping_active"
NAVIGATION_STATUS_TOPIC = "/robot/navigation_active"
CAMERA_STATUS_TOPIC = "/robot/camera_active"

DEFAULT_ROBOT_STATE = "idle"
NAVIGATION_READY_TIMEOUT_SECONDS = 5.0
NAVIGATION_READY_POLL_INTERVAL_SECONDS = 0.1
# INACTIVE_RESPONSE_TEXT, ALREADY_ACTIVE_RESPONSE_TEXT,
# NAVIGATION_SERVICE_CALL_TIMEOUT_SECONDS are imported from utils
SERVICE_CALL_TIMEOUT_SECONDS = 10.0  # Allow enough time for SLAM/Nav nodes to gracefully shutdown


class SystemController:
    def __init__(self, ros_client):
        self.ros = ros_client

        # System status topics
        self.state_sub = Topic(self.ros, ROBOT_STATE_TOPIC, STRING_MESSAGE_TYPE)
        self.mapping_status_sub = Topic(
            self.ros,
            MAPPING_STATUS_TOPIC,
            BOOL_MESSAGE_TYPE,
        )
        self.nav_status_sub = Topic(
            self.ros,
            NAVIGATION_STATUS_TOPIC,
            BOOL_MESSAGE_TYPE,
        )
        self.camera_status_sub = Topic(
            self.ros,
            CAMERA_STATUS_TOPIC,
            BOOL_MESSAGE_TYPE,
        )

        # System control services
        self.start_mapping_srv = Service(
            self.ros,
            START_MAPPING_SERVICE,
            TRIGGER_SERVICE_TYPE,
        )
        self.stop_mapping_srv = Service(
            self.ros,
            STOP_MAPPING_SERVICE,
            TRIGGER_SERVICE_TYPE,
        )
        self.save_map_srv = Service(self.ros, SAVE_MAP_SERVICE, TRIGGER_SERVICE_TYPE)
        self.start_nav_srv = Service(
            self.ros,
            START_NAVIGATION_SERVICE,
            TRIGGER_SERVICE_TYPE,
        )
        self.stop_nav_srv = Service(
            self.ros,
            STOP_NAVIGATION_SERVICE,
            TRIGGER_SERVICE_TYPE,
        )
        self.start_camera_srv = Service(
            self.ros,
            START_CAMERA_SERVICE,
            TRIGGER_SERVICE_TYPE,
        )
        self.stop_camera_srv = Service(
            self.ros,
            STOP_CAMERA_SERVICE,
            TRIGGER_SERVICE_TYPE,
        )

        # System state
        self.robot_state = DEFAULT_ROBOT_STATE
        self.mapping_active = False
        self.navigation_active = False
        self.camera_active = False
        # Reference to MotionController, wired by core.py after both controllers are created.
        # Used to keep motion._navigation_active in sync via _set_navigation_active().
        self._motion = None

        # Subscribe to status updates
        self.state_sub.subscribe(self._state_callback)
        self.mapping_status_sub.subscribe(self._mapping_callback)
        self.nav_status_sub.subscribe(self._nav_callback)
        self.camera_status_sub.subscribe(self._camera_callback)

    def _state_callback(self, msg):
        """Update robot state"""
        self.robot_state = msg["data"]

    def _mapping_callback(self, msg):
        """Update mapping status"""
        self.mapping_active = msg["data"]

    def _nav_callback(self, msg):
        """Update navigation status"""
        self.navigation_active = msg["data"]

    def _camera_callback(self, msg):
        """Update camera status"""
        self.camera_active = msg["data"]

    def _is_inactive_response(self, response):
        message = str(response.get("message", "")).lower()
        return INACTIVE_RESPONSE_TEXT in message

    def _wait_for_navigation_ready(self):
        start_time = time.time()
        while (time.time() - start_time) < NAVIGATION_READY_TIMEOUT_SECONDS:
            if self.is_navigating():
                return True
            time.sleep(NAVIGATION_READY_POLL_INTERVAL_SECONDS)
        return self.is_navigating()

    def start_mapping(self):
        """
        Start SLAM mapping mode.
        Called from: core.py → BonicBot.start_mapping()

        Returns:
            bool: True if mapping started successfully
        """
        try:
            call_trigger_service(
                self.start_mapping_srv,
                SERVICE_CALL_TIMEOUT_SECONDS,
                SystemControlError,
                "Failed to start mapping",
            )
            self.mapping_active = True
            print("🗺️ Mapping started - robot will create a map as it moves")
            return True
        except SystemControlError as exc:
            raise SystemControlError(f"Mapping start failed: {str(exc)}")

    def stop_mapping(self):
        """
        Stop SLAM mapping mode.
        Called from: core.py → BonicBot.stop_mapping()

        Returns:
            bool: True if mapping stopped successfully
        """
        if not self.mapping_active:
            return False

        try:
            call_trigger_service(
                self.stop_mapping_srv,
                SERVICE_CALL_TIMEOUT_SECONDS,
                SystemControlError,
                "Failed to stop mapping",
            )
            self.mapping_active = False
            print("🛑 Mapping stopped")
            return True
        except SystemControlError as exc:
            if INACTIVE_RESPONSE_TEXT in str(exc).lower():
                return False
            raise SystemControlError(f"Mapping stop failed: {str(exc)}")

    def save_map(self):
        """
        Save the current map.
        Called from: core.py → BonicBot.save_map()

        The map name is configured on the robot_manager ROS node side;
        std_srvs/Trigger does not carry a request payload.

        Returns:
            bool: True if map saved successfully
        """
        if not self.mapping_active:
            return False

        try:
            call_trigger_service(
                self.save_map_srv,
                SERVICE_CALL_TIMEOUT_SECONDS,
                SystemControlError,
                "Failed to save map",
            )
            print("💾 Map saved successfully")
            return True
        except SystemControlError as exc:
            if INACTIVE_RESPONSE_TEXT in str(exc).lower():
                return False
            raise SystemControlError(f"Map save failed: {str(exc)}")

    def start_navigation(self):
        """
        Start navigation mode (requires saved map or active mapping).
        Called from: core.py → BonicBot.start_navigation()

        Returns:
            bool: True if navigation started successfully
        """
        if self.is_mapping():
            return False

        try:
            call_trigger_service(
                self.start_nav_srv,
                NAVIGATION_SERVICE_CALL_TIMEOUT_SECONDS,
                SystemControlError,
                "Failed to start navigation",
            )
            # Set flags immediately after successful service call so that
            # downstream code (go_to, wait_for_goal) sees navigation as
            # active right away, without waiting for the ROS boolean topic
            # callback which may arrive late under heavy simulation load.
            self.navigation_active = True
            if self._motion is not None:
                self._motion._set_navigation_active(True)

            # Optional: wait for the Nav2 stack to fully initialise.
            # We already set the flag, so this is a best-effort confirmation.
            if not self._wait_for_navigation_ready():
                print("⚠️ Navigation service started but readiness "
                      "confirmation timed out — proceeding anyway")

            print("🧭 Navigation started - robot can now navigate to goals")
            return True
        except SystemControlError as exc:
            if ALREADY_ACTIVE_RESPONSE_TEXT in str(exc).lower():
                self.navigation_active = True
                if self._motion is not None:
                    self._motion._set_navigation_active(True)
                return True
            raise SystemControlError(f"Navigation start failed: {str(exc)}")

    def stop_navigation(self):
        """
        Stop navigation mode.
        Called from: core.py → BonicBot.stop_navigation()

        Returns:
            bool: True if navigation stopped successfully
        """
        try:
            call_trigger_service(
                self.stop_nav_srv,
                NAVIGATION_SERVICE_CALL_TIMEOUT_SECONDS,
                SystemControlError,
                "Failed to stop navigation",
            )
            self.navigation_active = False
            if self._motion is not None:
                self._motion._set_navigation_active(False)
            print("🛑 Navigation stopped")
            return True
        except SystemControlError as exc:
            if INACTIVE_RESPONSE_TEXT in str(exc).lower():
                self.navigation_active = False
                if self._motion is not None:
                    self._motion._set_navigation_active(False)
                return False
            raise SystemControlError(f"Navigation stop failed: {str(exc)}")

    def get_system_status(self):
        """
        Get current system status.
        Called from: core.py → BonicBot.get_system_status()

        Returns:
            dict: System status information
        """
        return {
            "state": self.robot_state,
            "mapping_active": self.mapping_active,
            "navigation_active": self.navigation_active,
            "camera_active": self.camera_active,
            "ready_for_goals": self.navigation_active,
        }

    def is_mapping(self):
        """
        Check if robot is currently mapping.
        Called from: core.py → BonicBot.is_mapping() | motion.py → _navigation_is_active() guard
        """
        return self.mapping_active

    def is_navigating(self):
        """
        Check if navigation system is active.
        Called from: core.py → BonicBot.is_navigating() | motion.py → _navigation_is_active()
        """
        return self.navigation_active

    def get_robot_state(self):
        """Get current robot state string."""
        return self.robot_state

    def start_camera(self):
        """
        Start camera system.
        Called from: core.py → BonicBot.system.start_camera() (direct user call)

        Returns:
            bool: True if camera started successfully
        """
        try:
            call_trigger_service(
                self.start_camera_srv,
                SERVICE_CALL_TIMEOUT_SECONDS,
                SystemControlError,
                "Failed to start camera",
            )
            print("📷 Camera started")
            return True
        except SystemControlError as exc:
            raise SystemControlError(f"Camera start failed: {str(exc)}")

    def stop_camera(self):
        """
        Stop camera system.
        Called from: core.py → BonicBot.system.stop_camera() (direct user call)

        Returns:
            bool: True if camera stopped successfully
        """
        try:
            call_trigger_service(
                self.stop_camera_srv,
                SERVICE_CALL_TIMEOUT_SECONDS,
                SystemControlError,
                "Failed to stop camera",
            )
            print("🛑 Camera stopped")
            return True
        except SystemControlError as exc:
            raise SystemControlError(f"Camera stop failed: {str(exc)}")

    def is_camera_active(self):
        """
        Check if camera system is active.
        Called from: core.py → BonicBot.system.is_camera_active() (direct user call)
        """
        return self.camera_active

    def setup_for_mapping(self):
        """
        Helper function to set up robot for mapping

        Returns:
            bool: True if setup successful
        """
        print("🔧 Setting up robot for mapping...")

        # Stop navigation if active
        if self.navigation_active:
            self.stop_navigation()
        
        # Start mapping
        return self.start_mapping()

    def setup_for_navigation(self):
        """
        Helper function to set up robot for autonomous navigation

        Returns:
            bool: True if setup successful
        """
        print("🔧 Setting up robot for navigation...")
        
        # Start navigation (will automatically check for saved map)
        return self.start_navigation()


    def shutdown(self):
        """Release system subscriptions during teardown."""
        for topic in (
            self.state_sub,
            self.mapping_status_sub,
            self.nav_status_sub,
            self.camera_status_sub,
        ):
            safe_unsubscribe(topic)
