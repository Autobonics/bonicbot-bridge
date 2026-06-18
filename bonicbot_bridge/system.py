"""
System controller for high-level robot operations
"""

import json
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
    ODOMETRY_MESSAGE_TYPE,
    POSE_STAMPED_MESSAGE_TYPE,
    FLOAT32_MESSAGE_TYPE,
    OCCUPANCY_GRID_MESSAGE_TYPE,
    MAP_TOPIC,
    MAP_AVAILABLE_TOPIC,
    START_MAPPING_SERVICE,
    STOP_MAPPING_SERVICE,
    SAVE_MAP_SERVICE,
    START_CAMERA_SERVICE,
    STOP_CAMERA_SERVICE,
    ROBOT_STATE_TOPIC,
    MAPPING_STATUS_TOPIC,
    NAVIGATION_STATUS_TOPIC,
    CAMERA_STATUS_TOPIC,
    GOTO_LOCATION_TOPIC,
    SAVE_LOCATION_TOPIC,
    DELETE_LOCATION_TOPIC,
    CURRENT_GOAL_TOPIC,
    LOCATIONS_LIST_TOPIC,
    ODOMETRY_FILTERED_TOPIC,
    call_trigger_service,
    safe_unsubscribe,
    safe_unadvertise,
)

# START_NAVIGATION_SERVICE and STOP_NAVIGATION_SERVICE are imported from utils

DEFAULT_ROBOT_STATE = "idle"
NAVIGATION_READY_TIMEOUT_SECONDS = 5.0
NAVIGATION_READY_POLL_INTERVAL_SECONDS = 0.1
DELETE_ALL_LOCATIONS_POLL_INTERVAL_SECONDS = 0.1
DELETE_ALL_LOCATIONS_TIMEOUT_SECONDS = 10.0
DELETE_ALL_LOCATIONS_SETTLE_SECONDS = 0.3
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

        # Cached map data — mirrors mappingreference.py's self.latest_map pattern.
        # The full OccupancyGrid dict is stored so the SDK can expose map metadata
        # (resolution, dimensions, origin) without requiring the dashboard.
        self.latest_map = None          # Full OccupancyGrid message dict
        self.map_info = None            # Extracted 'info' sub-dict for quick access

        # Track whether a saved map file exists on the robot's disk
        self.map_available = False
        self.map_available_sub = Topic(self.ros, MAP_AVAILABLE_TOPIC, BOOL_MESSAGE_TYPE)

        # Publishers for location management
        self.goto_loc_pub = Topic(self.ros, GOTO_LOCATION_TOPIC, STRING_MESSAGE_TYPE)
        self.save_loc_pub = Topic(self.ros, SAVE_LOCATION_TOPIC, STRING_MESSAGE_TYPE)
        self.delete_loc_pub = Topic(self.ros, DELETE_LOCATION_TOPIC, STRING_MESSAGE_TYPE)
        
        self.goto_loc_pub.advertise()
        self.save_loc_pub.advertise()
        self.delete_loc_pub.advertise()

        # Dynamic topics created on demand for streaming
        self._dynamic_topics = []

        # Single /map subscription shared by internal cache + dashboard.
        # throttle_rate=70ms ≈ 14 Hz — fast enough for live UI.
        self._map_cache_sub = Topic(self.ros, MAP_TOPIC, OCCUPANCY_GRID_MESSAGE_TYPE,
                                   throttle_rate=70)
        self._map_cache_sub.subscribe(self._map_data_callback)

        # Subscribe to status updates
        self.state_sub.subscribe(self._state_callback)
        self.mapping_status_sub.subscribe(self._mapping_callback)
        self.nav_status_sub.subscribe(self._nav_callback)
        self.camera_status_sub.subscribe(self._camera_callback)
        self.map_available_sub.subscribe(self._map_available_callback)

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

    def _map_data_callback(self, msg):
        """Cache the latest OccupancyGrid for SDK access.
        Mirrors mappingreference.py _map_callback (line 1207)."""
        self.latest_map = msg
        self.map_info = msg.get('info', None)

    def _map_available_callback(self, msg):
        """Track whether a saved map file exists on the robot."""
        self.map_available = msg["data"]

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
            # Reset cached map for new session (matches reference line 485)
            self.latest_map = None
            self.map_info = None
            print("🗺️ Mapping started - robot will create a map as it moves")
            return True
        except SystemControlError as exc:
            raise SystemControlError(f"Mapping start failed: {str(exc)}")

    def stop_mapping(self):
        """
        Stop SLAM mapping mode.
        Called from: core.py → BonicBot.stop_mapping()

        If navigation was running in SLAM mode (concurrent with mapping),
        the map is auto-saved and Nav2 is stopped, since it loses its map
        source when SLAM shuts down.

        Returns:
            bool: True if mapping stopped successfully
        """
        if not self.mapping_active:
            return False

        # If Nav2 was running in SLAM mode, auto-save map and stop Nav2
        # before killing SLAM (otherwise Nav2 loses its map source).
        if self.navigation_active:
            try:
                self.save_map()
                print("💾 Map auto-saved before stopping SLAM mode")
            except Exception:
                print("⚠️ Could not auto-save map before stopping mapping")
            self.stop_navigation()

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

    def start_navigation(self, force: bool = False):
        """
        Start navigation mode (requires saved map or active mapping).
        Called from: core.py → BonicBot.start_navigation()

        Args:
            force: If True, allow starting Nav2 even while mapping is active
                   (required for autonomous exploration / SLAM-mode navigation).

        Returns:
            bool: True if navigation started successfully
        """
        if self.is_mapping() and not force:
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

        # Clear cached map from any previous session
        self.latest_map = None
        self.map_info = None

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

    # --- Dashboard Publisher Wrappers ---
    
    def goto_location(self, name: str):
        """Send command to navigate to a saved location."""
        self.goto_loc_pub.publish({'data': name})
        
    def save_location(self, name: str):
        """Save the current robot pose as a named location."""
        self.save_loc_pub.publish({'data': name})
        
    def delete_location(self, name: str):
        """Delete a saved named location."""
        self.delete_loc_pub.publish({'data': name})

    def delete_all_locations(self) -> bool:
        _latest_locations: list[str] = []

        def _locations_callback(msg: dict) -> None:
            nonlocal _latest_locations
            raw = msg.get("data", "[]")
            try:
                parsed = json.loads(raw)
                _latest_locations = parsed if isinstance(parsed, list) else []
            except (json.JSONDecodeError, TypeError):
                _latest_locations = []

        # Temporary subscriber — only needed for the duration of this call
        locations_sub = self.subscribe_to_locations_list(_locations_callback)

        # Brief settle so the first callback fires before we start deleting
        time.sleep(DELETE_ALL_LOCATIONS_SETTLE_SECONDS)

        start_time = time.time()
        try:
            while True:
                if not _latest_locations:
                    print("🗑️ All locations deleted successfully")
                    return True

                if (time.time() - start_time) > DELETE_ALL_LOCATIONS_TIMEOUT_SECONDS:
                    print(
                        f"⚠️ delete_all_locations timed out — "
                        f"{len(_latest_locations)} location(s) still remain: "
                        f"{_latest_locations}"
                    )
                    return False

                # Delete every name currently visible in the list, then
                # re-read the topic to catch any that were missed
                for name in list(_latest_locations):
                    self.delete_location(name)
                    print(f"🗑️ Deleting location: '{name}'")

                # Wait for the robot to process deletes and publish an updated list
                time.sleep(DELETE_ALL_LOCATIONS_POLL_INTERVAL_SECONDS)
        finally:
            # Always clean up the temporary subscriber
            if locations_sub is not None:
                safe_unsubscribe(locations_sub)

    # --- Dashboard Streaming Subscription Wrappers ---

    def _create_dynamic_subscriber(self, topic_name, msg_type, callback, throttle_rate):
        """Helper to create and track a dynamic streaming topic."""
        topic = Topic(self.ros, topic_name, msg_type, throttle_rate=throttle_rate)
        topic.subscribe(callback)
        self._dynamic_topics.append(topic)
        return topic

    def subscribe_to_map(self, callback, throttle_rate=None):
        # Attach to the single shared /map subscription via ros.on()
        # (throttle_rate arg kept for signature compat but ignored — rate
        # is controlled by _map_cache_sub created in __init__)
        self.ros.on(MAP_TOPIC, callback)
        
    def subscribe_to_odom(self, callback, throttle_rate=100):
        return self._create_dynamic_subscriber(ODOMETRY_FILTERED_TOPIC, ODOMETRY_MESSAGE_TYPE, callback, throttle_rate)

    def subscribe_to_robot_state(self, callback):
        # roslibpy.Topic.subscribe() is a no-op when already subscribed,
        # so we use ros.on() directly to add extra callbacks.
        self.ros.on(ROBOT_STATE_TOPIC, callback)

    def subscribe_to_mapping_active(self, callback):
        self.ros.on(MAPPING_STATUS_TOPIC, callback)

    def subscribe_to_navigation_active(self, callback):
        self.ros.on(NAVIGATION_STATUS_TOPIC, callback)

    def subscribe_to_current_goal(self, callback, throttle_rate=500):
        return self._create_dynamic_subscriber(CURRENT_GOAL_TOPIC, POSE_STAMPED_MESSAGE_TYPE, callback, throttle_rate)



    def subscribe_to_locations_list(self, callback, throttle_rate=1000):
        return self._create_dynamic_subscriber(LOCATIONS_LIST_TOPIC, STRING_MESSAGE_TYPE, callback, throttle_rate)

    def subscribe_to_map_available(self, callback):
        self.ros.on(MAP_AVAILABLE_TOPIC, callback)



    # --- Map Data Access Methods ---

    def has_saved_map(self):
        """Check if a saved map file exists on the robot's disk.
        Mirrors mappingreference.py map_available_pub (line 210)."""
        return self.map_available

    def get_map_info(self):
        """Get metadata about the current map (resolution, width, height, origin).
        Returns None if no map data has been received yet."""
        return self.map_info

    def get_map_data(self):
        """Get the full cached OccupancyGrid message dict.
        Returns None if no map data has been received yet."""
        return self.latest_map

    def shutdown(self):
        """Release system subscriptions during teardown."""
        # Unadvertise publishers
        for pub in (self.goto_loc_pub, self.save_loc_pub, self.delete_loc_pub):
            safe_unadvertise(pub)

        for topic in (
            self.state_sub,
            self.mapping_status_sub,
            self.nav_status_sub,
            self.camera_status_sub,
            self.map_available_sub,
            self._map_cache_sub,
        ):
            safe_unsubscribe(topic)
            
        for topic in self._dynamic_topics:
            safe_unsubscribe(topic)
        self._dynamic_topics.clear()
