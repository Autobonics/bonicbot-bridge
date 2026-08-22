"""
Sensor manager for accessing robot sensor data
"""

import math
import time

from roslibpy import Topic

from .utils import (
    ODOMETRY_MESSAGE_TYPE,
    DIFF_CONT_ODOM_TOPIC,
    ODOMETRY_FILTERED_TOPIC,
    LASER_SCAN_MESSAGE_TYPE,
    LASER_SCAN_TOPIC,
    LASER_SCAN_THROTTLE_MS,
    IMU_MESSAGE_TYPE,
    IMU_TOPIC,
    IMU_THROTTLE_MS,
    safe_unsubscribe,
)

INITIAL_DATA_WAIT_SECONDS = 0.5
DEFAULT_BATTERY_PERCENT = 85.0
DEFAULT_SENSOR_TIMEOUT_SECONDS = 5
SENSOR_POLL_INTERVAL_SECONDS = 0.1
DEFAULT_POSITION_VALUE = 0.0


class SensorManager:
    def __init__(self, ros_client):
        self.ros = ros_client

        # Current sensor data
        self.current_pose = None
        self.battery_level = 0.0
        self.lidar_data = None
        self.imu_data = None
        self._position_sub = None

        # Subscribers (listen to both filtered and raw odometry)
        self.odom_sub = Topic(self.ros, ODOMETRY_FILTERED_TOPIC, ODOMETRY_MESSAGE_TYPE)
        self.raw_odom_sub = Topic(self.ros, DIFF_CONT_ODOM_TOPIC, ODOMETRY_MESSAGE_TYPE)
        self.scan_sub = Topic(
            self.ros,
            LASER_SCAN_TOPIC,
            LASER_SCAN_MESSAGE_TYPE,
            throttle_rate=LASER_SCAN_THROTTLE_MS,
        )
        self.imu_sub = Topic(
            self.ros, IMU_TOPIC, IMU_MESSAGE_TYPE, throttle_rate=IMU_THROTTLE_MS
        )

        # Start subscriptions
        self.odom_sub.subscribe(self._odom_callback)
        self.raw_odom_sub.subscribe(self._odom_callback)
        self.scan_sub.subscribe(self._scan_callback)
        self.imu_sub.subscribe(self._imu_callback)

        # Wait a moment for initial data
        time.sleep(INITIAL_DATA_WAIT_SECONDS)

    def _odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_pose = msg["pose"]["pose"]

    def _scan_callback(self, msg):
        """Cache the latest LaserScan message."""
        self.lidar_data = msg

    def _imu_callback(self, msg):
        """Cache the latest Imu message."""
        self.imu_data = msg

    def get_position(self):
        """
        Get current robot position

        Returns:
            dict: {'x': float, 'y': float, 'theta': float (degrees)} or None if no data
        """
        if not self.current_pose:
            return None

        position = self.current_pose["position"]
        orientation = self.current_pose["orientation"]

        # Convert quaternion to yaw angle (in radians first)
        theta_rad = 2 * math.atan2(orientation["z"], orientation["w"])

        # Convert to degrees
        theta_deg = math.degrees(theta_rad)

        return {
            "x": position["x"],
            "y": position["y"],
            "theta": theta_deg,
        }

    def get_x(self):
        """Get current X position in meters"""
        position = self.get_position()
        return position["x"] if position else DEFAULT_POSITION_VALUE

    def get_y(self):
        """Get current Y position in meters"""
        position = self.get_position()
        return position["y"] if position else DEFAULT_POSITION_VALUE

    def get_heading(self):
        """Get current robot heading in degrees"""
        position = self.get_position()
        return position["theta"] % 360 if position else DEFAULT_POSITION_VALUE

    def get_battery(self):
        """
        Get battery level percentage (0-100)
        Note: Implement based on your robot's battery topic
        """
        # TODO: Subscribe to actual battery topic when available
        # For now return a placeholder
        return DEFAULT_BATTERY_PERCENT

        #___________________________________________________________________________________________________________________#

    def get_lidar_scan(self):
        """
        Get the latest raw LiDAR scan.

        Returns:
            dict: Raw sensor_msgs/LaserScan message (angle_min, angle_max,
                  angle_increment, range_min, range_max, ranges[], intensities[])
                  or None if no scan has been received yet.
        """
        return self.lidar_data

    def get_min_obstacle_distance(self):
        """
        Get the closest obstacle distance from the latest LiDAR scan.

        Returns:
            float: Minimum range reading in meters, or None if no scan data
                   or all readings are out of range.
        """
        if not self.lidar_data:
            return None

        ranges = self.lidar_data.get("ranges", [])
        range_min = self.lidar_data.get("range_min", 0.0)
        range_max = self.lidar_data.get("range_max", float("inf"))

        valid = [
            r for r in ranges
            if isinstance(r, (int, float))
            and not math.isnan(r)
            and not math.isinf(r)
            and range_min <= r <= range_max
        ]
        return min(valid) if valid else None

    def get_imu_data(self):
        """
        Get the latest raw IMU message.

        Returns:
            dict: Raw sensor_msgs/Imu message (orientation, angular_velocity,
                  linear_acceleration, and their covariances) or None if no
                  IMU data has been received yet.
        """
        return self.imu_data

    def get_imu_orientation(self):
        """
        Get IMU-reported orientation as a quaternion.

        Returns:
            dict: {'x': float, 'y': float, 'z': float, 'w': float} or None
                  if no IMU data has been received yet.
        """
        if not self.imu_data:
            return None
        return self.imu_data.get("orientation")

    #_________________________________________________________________________________________________________________________________#
    def get_distance_traveled(self, start_pos=None):
        """
        Calculate distance traveled from a starting position

        Args:
            start_pos: Starting position dict {'x': float, 'y': float}
                      If None, returns 0

        Returns:
            float: Distance in meters
        """
        if not start_pos:
            return DEFAULT_POSITION_VALUE

        current = self.get_position()
        if not current:
            return DEFAULT_POSITION_VALUE

        delta_x = current["x"] - start_pos["x"]
        delta_y = current["y"] - start_pos["y"]
        return math.sqrt(delta_x * delta_x + delta_y * delta_y)

    def wait_for_data(
        self,
        timeout=DEFAULT_SENSOR_TIMEOUT_SECONDS,
        require_lidar=False,
        require_imu=False,
    ):
        """
        Wait for sensor data to become available

        Args:
            timeout: Maximum time to wait in seconds
            require_lidar: If True, also wait for LiDAR scan data
            require_imu: If True, also wait for IMU data

        Returns:
            bool: True if required data received, False on timeout
        """
        start_time = time.time()
    #_________________________________________________________________________________________________________________#
        while (time.time() - start_time) < timeout:
            has_pose = self.current_pose is not None
            has_lidar = (not require_lidar) or (self.lidar_data is not None)
            has_imu = (not require_imu) or (self.imu_data is not None)

            if has_pose and has_lidar and has_imu:
                return True
            time.sleep(SENSOR_POLL_INTERVAL_SECONDS)

        return False
    #_________________________________________________________________________________________________________________#
    def subscribe_to_position(self, callback):
        """
        Subscribe to position updates

        Args:
            callback: Function to call with position data
                     callback(x, y, theta)
        """
        def handle_position_update(_msg):
            position = self.get_position()
            if position:
                callback(position["x"], position["y"], position["theta"])

        if self._position_sub:
            self._position_sub.unsubscribe()

        self._position_sub = Topic(self.ros, DIFF_CONT_ODOM_TOPIC, ODOMETRY_MESSAGE_TYPE)
        self._position_sub.subscribe(handle_position_update)

    def get_sensor_info(self):
        """
        Get summary of all available sensor data

        Returns:
            dict: Summary of sensor states
        """
        position = self.get_position()

        return {
            "position": position,
            "battery": self.get_battery(),
            "sensors_active": position is not None,
            "lidar_active": self.lidar_data is not None,
            "imu_active": self.imu_data is not None,
            "timestamp": time.time(),
        }

    def shutdown(self):
        """Release sensor subscriptions during teardown."""
        for topic in (
            self._position_sub,
            self.odom_sub,
            self.raw_odom_sub,
            self.scan_sub,
            self.imu_sub,
        ):
            safe_unsubscribe(topic)
        self._position_sub = None
