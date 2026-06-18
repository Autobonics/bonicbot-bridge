"""
Motion controller for robot movement and navigation
"""

import math
import threading
import time

from roslibpy import Service, ServiceRequest, Topic

from .exceptions import NavigationError
from .precisemotion import QueueMixin
from .utils import (
    FLOAT32_MESSAGE_TYPE,
    NAVIGATION_SERVICE_CALL_TIMEOUT_SECONDS,
    POSE_STAMPED_MESSAGE_TYPE,
    POSE_WITH_COVARIANCE_MESSAGE_TYPE,
    STRING_MESSAGE_TYPE,
    TWIST_MESSAGE_TYPE,
    CMD_VEL_TOPIC,
    GOAL_POSE_TOPIC,
    INITIAL_POSE_TOPIC,
    NAV_STATUS_TOPIC,
    DISTANCE_TO_GOAL_TOPIC,
    CANCEL_NAVIGATION_SERVICE,
    safe_unsubscribe,
)

# START_NAVIGATION_SERVICE and STOP_NAVIGATION_SERVICE are imported from utils

DEFAULT_LINEAR_SPEED = 0.3
DEFAULT_TURN_SPEED = 0.5
DEFAULT_GOAL_TIMEOUT_SECONDS = 30
CMD_VEL_PUBLISH_RATE_HZ = 10
STATUS_POLL_INTERVAL_SECONDS = 0.1
INITIAL_POSE_TOPIC_READY_DELAY_SECONDS = 0.15
INITIAL_POSE_PUBLISH_SETTLE_DELAY_SECONDS = 0.2
# NAVIGATION_SERVICE_CALL_TIMEOUT_SECONDS, INACTIVE_RESPONSE_TEXT,
# ALREADY_ACTIVE_RESPONSE_TEXT are imported from utils

MAP_FRAME_ID = "map"
NAV_STATUS_IDLE = "inactive"
NAV_STATUS_NAVIGATING = "navigating"
NAV_STATUS_GOAL_REACHED = "goal_reached"
NAV_STATUS_GOAL_FAILED = "goal_failed"
NAV_STATUS_CANCELLED = "cancelled"
NAV_STATUS_TIMEOUT = "timeout"
ROBOT_MANAGER_NAV_STATUS_IDLE = "idle"
NO_ACTIVE_GOAL_MESSAGE = "no active navigation goal"
TERMINAL_NAV_STATUSES = (
    NAV_STATUS_GOAL_REACHED,
    NAV_STATUS_GOAL_FAILED,
    NAV_STATUS_CANCELLED,
)

GOAL_STAMP_SECONDS = 0
GOAL_STAMP_NANOSECONDS = 0
NANOSECONDS_PER_SECOND = 1_000_000_000
POSE_COVARIANCE_LENGTH = 36


class MotionController(QueueMixin):
    def __init__(self, ros_client):
        self.ros = ros_client

        # Movement publisher
        self.cmd_vel_pub = Topic(self.ros, CMD_VEL_TOPIC, TWIST_MESSAGE_TYPE)
        self.cmd_vel_pub.advertise()

        # Navigation goal publisher and status subscribers
        self.goal_pub = Topic(self.ros, GOAL_POSE_TOPIC, POSE_STAMPED_MESSAGE_TYPE)
        self.goal_pub.advertise()
        self.nav_status_sub = Topic(self.ros, NAV_STATUS_TOPIC, STRING_MESSAGE_TYPE)
        self.distance_sub = Topic(
            self.ros, DISTANCE_TO_GOAL_TOPIC, FLOAT32_MESSAGE_TYPE
        )

        # Cancel-navigation service (start/stop nav services live in SystemController)
        self.cancel_nav_srv = Service(
            self.ros,
            CANCEL_NAVIGATION_SERVICE,
            "std_srvs/Trigger",
        )

        # State tracking — protected by _state_lock for thread-safe access
        # between main thread and roslibpy callback threads.
        self._state_lock = threading.Lock()
        self._goal_done_event = threading.Event()
        self.nav_status = NAV_STATUS_IDLE
        self.distance_to_goal = None
        self._goal_active = False
        self._navigation_active = False
        self._move_cancel = threading.Event()
        # system is set by core.py after both controllers are created
        self.system = None
        # Subscribe to status updates
        self.nav_status_sub.subscribe(self._nav_status_callback)
        self.distance_sub.subscribe(self._distance_callback)

        # Initialise the command queue (from QueueMixin)
        self._init_queue()

    def _normalize_nav_status(self, status):
        if status == ROBOT_MANAGER_NAV_STATUS_IDLE:
            return NAV_STATUS_IDLE
        return status

    def _navigation_is_active(self):
        if self.system is not None:
            return self.system.is_navigating()
        return self._navigation_active

    def _set_navigation_active(self, active):
        """
        Update the local navigation-active flag.
        Called from: system.py → SystemController.start_navigation() / stop_navigation()
        """
        self._navigation_active = active
        if not active:
            self._clear_goal_state(NAV_STATUS_IDLE)

    def _clear_goal_state(self, status=None):
        with self._state_lock:
            self._goal_active = False
            self.distance_to_goal = None
            if status is not None:
                self.nav_status = status

    def _nav_status_callback(self, msg):
        """Update navigation status"""
        raw = self._normalize_nav_status(msg["data"])
        with self._state_lock:
            # Don't let a stale 'idle' overwrite a just-published goal
            if self._goal_active and raw == NAV_STATUS_IDLE:
                return
            self.nav_status = raw
            # Signal wait_for_goal if we reached a terminal state
            if raw in TERMINAL_NAV_STATUSES:
                self._goal_done_event.set()

    def _distance_callback(self, msg):
        """Update distance to goal"""
        distance = msg["data"]
        with self._state_lock:
            if self._goal_active and distance >= 0:
                self.distance_to_goal = distance

    def _move_for_duration(self, duration, linear_x=0, linear_y=0, angular_z=0):
        interval = 1.0 / CMD_VEL_PUBLISH_RATE_HZ
        start_time = time.time()
        self._move_cancel.clear()

        while (time.time() - start_time) < duration and not self._move_cancel.is_set():
            self.move(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z)
            time.sleep(interval)

        self.stop()

    def move(self, linear_x=0, linear_y=0, angular_z=0):
        """
        Send velocity command to robot.
        Called from: core.py → BonicBot (no direct delegate; used internally)

        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s) - for omnidirectional robots
            angular_z: Rotational velocity (deg/s)
        """
        # Convert angular velocity from deg/s to rad/s for ROS
        angular_z_rad = math.radians(angular_z)

        msg = {
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z_rad},
        }
        self.cmd_vel_pub.publish(msg)

    def move_forward(self, speed=DEFAULT_LINEAR_SPEED, duration=None):
        """
        Move robot forward.
        Called from: core.py → BonicBot.move_forward()

        Args:
            speed: Forward speed in m/s (default: 0.3)
            duration: Time to move in seconds (None for continuous)
        """
        if duration:
            self._move_for_duration(duration, linear_x=speed)
            return

        # Continuous movement (single command)
        self.move(linear_x=speed)

    def move_backward(self, speed=DEFAULT_LINEAR_SPEED, duration=None):
        """
        Move robot backward.
        Called from: core.py → BonicBot.move_backward()
        """
        if duration:
            self._move_for_duration(duration, linear_x=-speed)
            return

        # Continuous movement (single command)
        self.move(linear_x=-speed)

    def turn_left(self, speed=DEFAULT_TURN_SPEED, duration=None):
        """
        Turn robot left (counter-clockwise).
        Called from: core.py → BonicBot.turn_left()
        """
        if duration:
            self._move_for_duration(duration, angular_z=speed)
            return

        # Continuous movement (single command)
        self.move(angular_z=speed)

    def turn_right(self, speed=DEFAULT_TURN_SPEED, duration=None):
        """
        Turn robot right (clockwise).
        Called from: core.py → BonicBot.turn_right()
        """
        if duration:
            self._move_for_duration(duration, angular_z=-speed)
            return

        # Continuous movement (single command)
        self.move(angular_z=-speed)

    def stop(self):
        """
        Stop all robot movement.
        Called from: core.py → BonicBot.stop() and BonicBot.disconnect()
        """
        self._move_cancel.set()
        self.move(0, 0, 0)

    def go_to(self, x, y, theta=0):
        """
        Navigate to specific coordinate using Nav2.
        Called from: core.py → BonicBot.go_to()

        Args:
            x: Target X coordinate (meters)
            y: Target Y coordinate (meters)
            theta: Target orientation (degrees, default: 0)

        Returns:
            bool: True if goal was sent successfully
        """
        if not self._navigation_is_active():
            self._clear_goal_state(NAV_STATUS_IDLE)
            return False

        try:
            # Convert degrees to radians for ROS message
            theta_rad = math.radians(theta)

            # Create goal message
            goal_msg = {
                "header": {
                    "stamp": {
                        "sec": GOAL_STAMP_SECONDS,
                        "nanosec": GOAL_STAMP_NANOSECONDS,
                    },
                    "frame_id": MAP_FRAME_ID,
                },
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": math.sin(theta_rad / 2),
                        "w": math.cos(theta_rad / 2),
                    },
                },
            }

            # Publish goal
            self.goal_pub.publish(goal_msg)
            with self._state_lock:
                self.nav_status = NAV_STATUS_NAVIGATING
                self._goal_active = True
                self.distance_to_goal = None
                self._goal_done_event.clear()
            print(f"🎯 Navigation goal set: ({x:.2f}, {y:.2f}, θ={theta:.1f}°)")
            return True

        except Exception as exc:
            raise NavigationError(f"Failed to set navigation goal: {str(exc)}")

    def cancel_goal(self):
        """
        Cancel current navigation goal.
        Called from: core.py → BonicBot.cancel_goal()
        """
        if not self._goal_active:
            self.distance_to_goal = None
            return False

        try:
            request = ServiceRequest()
            response = self.cancel_nav_srv.call(
                request, timeout=NAVIGATION_SERVICE_CALL_TIMEOUT_SECONDS
            )
            if not response["success"]:
                raise NavigationError(f"Failed to cancel goal: {response['message']}")
        except NavigationError as exc:
            if NO_ACTIVE_GOAL_MESSAGE in str(exc).lower():
                self._clear_goal_state(NAV_STATUS_IDLE)
                return False
            raise
        except Exception as exc:
            raise NavigationError(f"Failed to cancel goal: {str(exc)}")

        self._clear_goal_state(NAV_STATUS_CANCELLED)
        print("❌ Navigation goal cancelled")
        return True

    def set_initial_pose(self, x, y, theta=0):
        """
        Set initial pose for robot localization.
        Called from: core.py → BonicBot.set_initial_pose()

        Args:
            x: Initial X coordinate (meters)
            y: Initial Y coordinate (meters)
            theta: Initial orientation (degrees, default: 0)

        Returns:
            bool: True if pose was set successfully
        """
        if self._goal_active:
            return False

        initial_pose_pub = None
        try:
            # Convert degrees to radians for ROS message
            theta_rad = math.radians(theta)

            # Create valid covariance array
            cov = [0.0] * POSE_COVARIANCE_LENGTH
            cov[0] = 0.25  # X variance
            cov[7] = 0.25  # Y variance
            cov[35] = 0.068  # Yaw variance

            # Create initial pose topic
            initial_pose_pub = Topic(
                self.ros, INITIAL_POSE_TOPIC, POSE_WITH_COVARIANCE_MESSAGE_TYPE
            )

            initial_pose_pub.advertise()
            time.sleep(INITIAL_POSE_TOPIC_READY_DELAY_SECONDS)

            # Create pose message
            pose_msg = {
                "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": MAP_FRAME_ID},
                "pose": {
                    "pose": {
                        "position": {"x": x, "y": y, "z": 0.0},
                        "orientation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": math.sin(theta_rad / 2),
                            "w": math.cos(theta_rad / 2),
                        },
                    },
                    "covariance": cov,
                },
            }

            # Publish initial pose
            initial_pose_pub.publish(pose_msg)
            print(f"📍 Initial pose set: ({x:.2f}, {y:.2f}, θ={theta:.1f}°)")

            time.sleep(INITIAL_POSE_PUBLISH_SETTLE_DELAY_SECONDS)

            return True

        except Exception as exc:
            raise NavigationError(f"Failed to set initial pose: {str(exc)}")
        finally:
            if initial_pose_pub is not None:
                try:
                    initial_pose_pub.unadvertise()
                except Exception:
                    pass

    def wait_for_goal(self, timeout=DEFAULT_GOAL_TIMEOUT_SECONDS):
        """
        Wait for current navigation goal to complete.
        Called from: core.py → BonicBot.wait_for_goal()

        Uses a threading.Event signal from the status callback instead of
        polling, eliminating timing-related missed-status races.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            str: Final navigation status ('goal_reached', 'goal_failed', 'cancelled')
        """
        with self._state_lock:
            if not self._goal_active:
                print("⏰ Navigation timeout: no active goal")
                return NAV_STATUS_TIMEOUT

        # Wait for the callback to signal a terminal status
        goal_completed = self._goal_done_event.wait(timeout=timeout)

        with self._state_lock:
            if goal_completed and self.nav_status in TERMINAL_NAV_STATUSES:
                if self.nav_status == NAV_STATUS_GOAL_REACHED:
                    print("✅ Goal reached!")
                elif self.nav_status == NAV_STATUS_GOAL_FAILED:
                    print("❌ Goal failed!")
                else:
                    print("🚫 Goal cancelled!")
                final_status = self.nav_status
            else:
                print(f"⏰ Navigation timeout after {timeout}s")
                final_status = NAV_STATUS_TIMEOUT

        self._clear_goal_state(final_status)
        return final_status

    def get_nav_status(self):
        """
        Get current navigation status.
        Called from: core.py → BonicBot.get_nav_status()
        """
        if not self._navigation_is_active() and not self._goal_active:
            return NAV_STATUS_IDLE
        return self.nav_status

    def get_distance_to_goal(self):
        """
        Get distance to current navigation goal in meters.
        Called from: core.py → BonicBot.get_distance_to_goal()
        """
        if not self._goal_active or self.nav_status != NAV_STATUS_NAVIGATING:
            return None
        return self.distance_to_goal

    def is_moving(self):
        """
        Check if robot is currently moving.
        Called from: core.py → BonicBot.is_moving()
        """
        return self._goal_active and self.nav_status == NAV_STATUS_NAVIGATING

    def subscribe_to_nav_status(self, callback):
        # roslibpy.Topic.subscribe() is a no-op when already subscribed,
        # so we use ros.on() directly to add extra callbacks.
        self.ros.on(NAV_STATUS_TOPIC, callback)

    def subscribe_to_distance_to_goal(self, callback):
        self.ros.on(DISTANCE_TO_GOAL_TOPIC, callback)

    def shutdown(self):
        """Release motion subscriptions during teardown."""
        # Shut down the command queue first (from QueueMixin)
        try:
            self._queue_shutdown()
        except Exception as exc:
            print(f"⚠️ Error shutting down queue during shutdown: {exc}")

        try:
            self.stop()
        except Exception as exc:
            print(f"⚠️ Error stopping motion during shutdown: {exc}")

        for pub in (self.goal_pub, self.cmd_vel_pub):
            try:
                pub.unadvertise()
            except Exception as exc:
                print(f"⚠️ Error unadvertising publisher during shutdown: {exc}")

        for topic in (self.nav_status_sub, self.distance_sub):
            safe_unsubscribe(topic)
