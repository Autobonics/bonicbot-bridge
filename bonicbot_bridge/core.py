"""
Core BonicBot class for robot control
"""

import time

from roslibpy import Ros
from twisted.internet.error import ReactorNotRunning

from .camera import CameraManager
from .exceptions import ConnectionError
from .motion import MotionController
from .sensors import SensorManager
from .servo import ServoController
from .system import SystemController
from .vision import VisionController
from .autonomous import ExploreController

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 9090
DEFAULT_CONNECTION_TIMEOUT_SECONDS = 10
CONNECTION_POLL_INTERVAL_SECONDS = 0.1
DEFAULT_LINEAR_SPEED = 0.3
DEFAULT_TURN_SPEED = 0.5
DEFAULT_GOAL_TIMEOUT_SECONDS = 30

_reactor_stopped = False


class BonicBot:
    def __init__(
        self,
        host=DEFAULT_HOST,
        port=DEFAULT_PORT,
        timeout=DEFAULT_CONNECTION_TIMEOUT_SECONDS,
    ):
        """
        Initialize BonicBot connection

        Args:
            host: Robot IP address or hostname (default: localhost)
            port: rosbridge port (default: 9090)
            timeout: Connection timeout in seconds
        """
        self.host = host
        self.port = port
        self.ros = None
        self.connected = False

        # Controllers
        self.motion = None
        self.sensors = None
        self.system = None
        self._camera = None
        self.servo = None
        self.vision = None
        self.explore = None

        # Connect to robot
        self.connect(timeout)

    def connect(self, timeout=DEFAULT_CONNECTION_TIMEOUT_SECONDS):
        """Establish connection to robot"""
        if timeout <= 0:
            raise ValueError("Connection timeout must be greater than 0")

        if self.connected:
            return True

        try:
            self.ros = Ros(host=self.host, port=self.port)
            self.ros.run()

            # Wait for connection
            start_time = time.time()
            while not self.ros.is_connected and (time.time() - start_time) < timeout:
                time.sleep(CONNECTION_POLL_INTERVAL_SECONDS)

            if not self.ros.is_connected:
                raise ConnectionError(
                    f"Failed to connect to robot at {self.host}:{self.port}"
                )

            # Initialize controllers
            self.motion = MotionController(self.ros)
            self.sensors = SensorManager(self.ros)
            self.system = SystemController(self.ros)
            self.motion.system = self.system
            # Wire motion reference so system can keep the nav-active flag in sync
            self.system._motion = self.motion
            self._camera = None
            self.servo = ServoController(self.ros)
            self.vision = VisionController(self.ros)
            self.explore = ExploreController(self.ros, self.system)

            self.connected = True
            print(f"🤖 Connected to BonicBot at {self.host}:{self.port}")
            return True

        except Exception as exc:
            raise ConnectionError(f"Connection failed: {str(exc)}")

    def disconnect(self):
        """Disconnect from robot"""
        if self.ros and self.ros.is_connected:
            for controller in (
                self.motion,
                self.sensors,
                self.system,
                self._camera,
                self.servo,
                self.vision,
                self.explore,
            ):
                if not controller:
                    continue
                try:
                    if hasattr(controller, "shutdown"):
                        controller.shutdown()
                except Exception as exc:
                    print(f"⚠️ Error during controller shutdown: {exc}")

            global _reactor_stopped
            try:
                self.ros.terminate()
            except ReactorNotRunning:
                if not _reactor_stopped:
                    print("⚠️ reactor already stopped")
                    _reactor_stopped = True

            self.connected = False
            print("🔌 Disconnected from BonicBot")

    def _get_camera(self):
        if self._camera is None:
            self._camera = CameraManager(self.ros)
        return self._camera

    @property
    def camera(self):
        return self._get_camera()

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()

    # Quick access methods (delegate to controllers)
    def move(self, linear_x=0, linear_y=0, angular_z=0):
        """Publish raw velocity commands. Delegates to: motion.move()"""
        return self.motion.move(linear_x, linear_y, angular_z)

    def move_forward(self, speed=DEFAULT_LINEAR_SPEED, duration=None):
        """Move robot forward. Delegates to: motion.move_forward()"""
        return self.motion.move_forward(speed, duration)

    def move_backward(self, speed=DEFAULT_LINEAR_SPEED, duration=None):
        """Move robot backward. Delegates to: motion.move_backward()"""
        return self.motion.move_backward(speed, duration)

    def turn_left(self, speed=DEFAULT_TURN_SPEED, duration=None):
        """Turn robot left. Delegates to: motion.turn_left()"""
        return self.motion.turn_left(speed, duration)

    def turn_right(self, speed=DEFAULT_TURN_SPEED, duration=None):
        """Turn robot right. Delegates to: motion.turn_right()"""
        return self.motion.turn_right(speed, duration)

    def stop(self):
        """Stop robot movement. Delegates to: motion.stop()"""
        return self.motion.stop()

    def go_to(self, x, y, theta=0):
        """Navigate to specific coordinate. Delegates to: motion.go_to()"""
        return self.motion.go_to(x, y, theta)

    def get_battery(self):
        """Get battery percentage. Delegates to: sensors.get_battery()"""
        if not self.is_connected():
            return None
        return self.sensors.get_battery()

    def get_position(self):
        """Get current robot position. Delegates to: sensors.get_position()"""
        return self.sensors.get_position()

    def get_x(self):
        """Get current X position in meters. Delegates to: sensors.get_x()"""
        return self.sensors.get_x()

    def get_y(self):
        """Get current Y position in meters. Delegates to: sensors.get_y()"""
        return self.sensors.get_y()

    def get_heading(self):
        """Get current robot heading in degrees. Delegates to: sensors.get_heading()"""
        return self.sensors.get_heading()

    def get_distance_traveled(self, start_pos=None):
        """Get distance traveled since start or from a given position. Delegates to: sensors.get_distance_traveled()"""
        return self.sensors.get_distance_traveled(start_pos)

    def wait_for_data(self, timeout=5.0):
        """Wait for initial sensor data. Delegates to: sensors.wait_for_data()"""
        return self.sensors.wait_for_data(timeout)

    def subscribe_to_position(self, callback):
        """Subscribe to continuous position updates. Delegates to: sensors.subscribe_to_position()"""
        return self.sensors.subscribe_to_position(callback)

    def get_sensor_info(self):
        """Get all sensor state. Delegates to: sensors.get_sensor_info()"""
        return self.sensors.get_sensor_info()

    def start_mapping(self):
        """Start mapping mode. Delegates to: system.start_mapping()"""
        return self.system.start_mapping()

    def stop_mapping(self):
        """Stop mapping mode. Delegates to: system.stop_mapping()"""
        return self.system.stop_mapping()

    def save_map(self):
        """Save current map. Delegates to: system.save_map()"""
        return self.system.save_map()

    def start_navigation(self):
        """Start navigation system. Delegates to: system.start_navigation()"""
        return self.system.start_navigation()

    def stop_navigation(self):
        """Stop navigation system. Delegates to: system.stop_navigation()"""
        return self.system.stop_navigation()

    def cancel_goal(self):
        """Cancel current navigation goal. Delegates to: motion.cancel_goal()"""
        return self.motion.cancel_goal()

    def get_nav_status(self):
        """Get current navigation status. Delegates to: motion.get_nav_status()"""
        return self.motion.get_nav_status()

    def is_moving(self):
        """Check if robot is currently moving. Delegates to: motion.is_moving()"""
        return self.motion.is_moving()

    def get_system_status(self):
        """Get system status information. Delegates to: system.get_system_status()"""
        return self.system.get_system_status()

    def is_mapping(self):
        """Check if robot is currently mapping. Delegates to: system.is_mapping()"""
        return self.system.is_mapping()

    def is_navigating(self):
        """Check if navigation system is active. Delegates to: system.is_navigating()"""
        return self.system.is_navigating()

    def get_robot_state(self):
        """Get current robot hardware state. Delegates to: system.get_robot_state()"""
        return self.system.get_robot_state()

    def activate_camera_hardware(self):
        """Turn on the physical camera node on the robot. Delegates to: system.start_camera()"""
        return self.system.start_camera()

    def deactivate_camera_hardware(self):
        """Turn off the physical camera node on the robot. Delegates to: system.stop_camera()"""
        return self.system.stop_camera()

    def is_camera_active(self):
        """Check if physical camera node is running. Delegates to: system.is_camera_active()"""
        return self.system.is_camera_active()

    def setup_for_mapping(self):
        """Configure system for mapping. Delegates to: system.setup_for_mapping()"""
        return self.system.setup_for_mapping()

    def has_saved_map(self):
        """Check if a saved map exists on the robot. Delegates to: system.has_saved_map()"""
        return self.system.has_saved_map()

    def get_map_info(self):
        """Get current map metadata (resolution, width, height, origin). Delegates to: system.get_map_info()"""
        return self.system.get_map_info()

    def get_map_data(self):
        """Get the full cached OccupancyGrid map data. Delegates to: system.get_map_data()"""
        return self.system.get_map_data()

    def setup_for_navigation(self):
        """Configure system for navigation. Delegates to: system.setup_for_navigation()"""
        return self.system.setup_for_navigation()

    def setup_for_exploration(self):
        """Configure system for autonomous exploration. Delegates to: explore.setup_for_exploration()"""
        return self.explore.setup_for_exploration()

    def start_explore(self):
        """Start autonomous exploration. Delegates to: explore.start_explore()"""
        return self.explore.start_explore()

    def stop_explore(self):
        """Stop autonomous exploration. Delegates to: explore.stop_explore()"""
        return self.explore.stop_explore()

    def is_exploring(self):
        """Check if autonomous exploration is active. Delegates to: explore.is_exploring()"""
        return self.explore.is_exploring()

    def get_explored_area(self):
        """Get the current explored area in square meters. Delegates to: explore.get_explored_area()"""
        return self.explore.get_explored_area()

    def wait_for_map_complete(self, min_area, timeout=300.0):
        """Wait for exploration to complete. Delegates to: explore.wait_for_map_complete()"""
        return self.explore.wait_for_map_complete(min_area, timeout)

    def is_connected(self):
        """Check if connected to robot."""
        return self.connected and self.ros and self.ros.is_connected

    def wait_for_goal(self, timeout=DEFAULT_GOAL_TIMEOUT_SECONDS):
        """Wait for current navigation goal to complete. Delegates to: motion.wait_for_goal()"""
        return self.motion.wait_for_goal(timeout)

    def get_distance_to_goal(self):
        """Get distance to current navigation goal. Delegates to: motion.get_distance_to_goal()"""
        return self.motion.get_distance_to_goal()

    def set_initial_pose(self, x, y, theta=0):
        """Set initial pose for localization. Delegates to: motion.set_initial_pose()"""
        return self.motion.set_initial_pose(x, y, theta)

    # Camera methods
    def start_camera(self, callback=None):
        """
        Start camera streaming (client-side subscription).
        Delegates to: camera.start_streaming()

        Note: Call bot.system.start_camera() first to activate robot's camera hardware,
        then call this to start receiving images in your script.

        Args:
            callback: Optional function(image) called for each frame
        """
        return self._get_camera().start_streaming(callback=callback)

    def stop_camera(self):
        """
        Stop camera streaming (client-side subscription).
        Delegates to: camera.stop_streaming()

        Note: Call bot.system.stop_camera() after this to deactivate robot's camera
        hardware for better performance.
        """
        if not self._camera:
            return False
        return self._camera.stop_streaming()

    def get_image(self):
        """Get latest camera image. Delegates to: camera.get_latest_image()"""
        if not self._camera:
            return None
        return self._camera.get_latest_image()

    def save_image(self, filepath):
        """Save current camera image. Delegates to: camera.save_image()"""
        if not self._camera:
            return False
        return self._camera.save_image(filepath)

    def get_camera_info(self):
        """Get camera intrinsics/info. Delegates to: camera.get_camera_info()"""
        return self._camera.get_camera_info() if self._camera else None

    def is_streaming(self):
        """Check if camera stream is active locally. Delegates to: camera.is_streaming()"""
        return self._camera.is_streaming() if self._camera else False

    def wait_for_image(self, timeout=5.0):
        """Wait for the next image frame. Delegates to: camera.wait_for_image()"""
        return self._camera.wait_for_image(timeout) if self._camera else False

    # Servo shortcuts
    def set_servos(self, angles):
        """Set servo angles (dictionary of joint_name: angle_degrees). Delegates to: servo.set_servo_angles()"""
        return self.servo.set_servo_angles(angles)

    def move_left_arm(self, shoulder, elbow, wait=True):
        """Move left arm (shoulder, elbow angles in degrees). Delegates to: servo.move_left_arm()"""
        return self.servo.move_left_arm(shoulder, elbow, wait=wait)

    def move_right_arm(self, shoulder, elbow, wait=True):
        """Move right arm (shoulder, elbow angles in degrees). Delegates to: servo.move_right_arm()"""
        return self.servo.move_right_arm(shoulder, elbow, wait=wait)

    def set_grippers(self, left, right):
        """Set gripper angles in degrees. Delegates to: servo.set_grippers()"""
        return self.servo.set_grippers(left, right)

    def open_grippers(self):
        """Open both grippers. Delegates to: servo.open_grippers()"""
        return self.servo.open_grippers()

    def close_grippers(self):
        """Close both grippers. Delegates to: servo.close_grippers()"""
        return self.servo.close_grippers()

    def set_neck(self, yaw):
        """Set neck yaw angle in degrees. Delegates to: servo.set_neck()"""
        return self.servo.set_neck(yaw)

    def look_left(self):
        """Turn neck fully left. Delegates to: servo.look_left()"""
        return self.servo.look_left()

    def look_right(self):
        """Turn neck fully right. Delegates to: servo.look_right()"""
        return self.servo.look_right()

    def look_center(self):
        """Center the neck. Delegates to: servo.look_center()"""
        return self.servo.look_center()

    def reset_servos(self):
        """Reset all servos to neutral position. Delegates to: servo.reset_all_servos()"""
        return self.servo.reset_all_servos()

    def set_left_gripper(self, angle):
        """Set left gripper angle. Delegates to: servo.set_left_gripper()"""
        return self.servo.set_left_gripper(angle)

    def set_right_gripper(self, angle):
        """Set right gripper angle. Delegates to: servo.set_right_gripper()"""
        return self.servo.set_right_gripper(angle)

    def get_servo_angles(self):
        """Get current angles of all servos. Delegates to: servo.get_servo_angles()"""
        return self.servo.get_servo_angles()

    def get_servo_limits(self):
        """Get angle limits for all servos. Delegates to: servo.get_servo_limits()"""
        return self.servo.get_servo_limits()

    def set_single_servo(self, joint_name, angle):
        """Set a specific servo by name. Delegates to: servo.set_single_servo()"""
        return self.servo.set_single_servo(joint_name, angle)

    def get_single_servo(self, joint_name):
        """Get angle of a specific servo. Delegates to: servo.get_single_servo()"""
        return self.servo.get_single_servo(joint_name)

        # ═══════════════════════════════════════════════════════════════════════

    # NEW — Precise-motion delegates (drive_distance / rotate_angle / drive_and_rotate)
    # ═══════════════════════════════════════════════════════════════════════

    def drive_distance(self, dist, speed=0.3, engine="internal", timeout=30.0):
        """Drive a specific distance. Delegates to: motion.drive_distance()"""
        return self.motion.drive_distance(dist, speed, engine=engine, timeout=timeout)

    def rotate_angle(self, angle, speed=45.0, engine="internal", timeout=30.0):
        """Rotate by a specific angle. Delegates to: motion.rotate_angle()"""
        return self.motion.rotate_angle(angle, speed, engine=engine, timeout=timeout)

    def drive_and_rotate(
        self, dist, angle, speed=0.3, turn_speed=45.0, engine="internal", timeout=30.0
    ):
        """Drive then rotate. Delegates to: motion.drive_and_rotate()"""
        return self.motion.drive_and_rotate(
            dist, angle, speed, turn_speed, engine=engine, timeout=timeout
        )

    def set_default_engine(self, engine):
        """Set default precise motion engine ('internal' or 'nav2'). Delegates to: motion.set_default_engine()"""
        return self.motion.set_default_engine(engine)

    def is_precise_moving(self):
        """Check if a precise motion action is running. Delegates to: motion.is_precise_moving()"""
        return self.motion.is_precise_moving()

    # ═══════════════════════════════════════════════════════════════════════

    # NEW — Command queue delegates (enqueue_move / run_queue / clear_queue / draw_square)
    # ═══════════════════════════════════════════════════════════════════════

    def enqueue_move(self, cmd_list):
        """Push motion commands onto the queue. Delegates to: motion.enqueue_move()"""
        return self.motion.enqueue_move(cmd_list)

    def run_queue(self, block=True):
        """Execute queued commands. Delegates to: motion.run_queue()"""
        return self.motion.run_queue(block)

    def clear_queue(self):
        """Flush queue and stop current motion. Delegates to: motion.clear_queue()"""
        return self.motion.clear_queue()

    def draw_square(self, side_m, speed=0.3, turn_speed=45.0,
                    engine="internal", timeout=30.0):
        """Drive in a square pattern. Delegates to: motion.draw_square()"""
        return self.motion.draw_square(
            side_m, speed=speed, turn_speed=turn_speed,
            engine=engine, timeout=timeout
        )

    # ═══════════════════════════════════════════════════════════════════════

    # NEW — Vision pipeline delegates (enable_detection / disable_detection / get_active_mode)
    # ═══════════════════════════════════════════════════════════════════════

    def enable_detection(self, mode, model='yolov8n', **kwargs):
        """Enable a vision detection mode. Delegates to: vision.enable_detection()"""
        return self.vision.enable_detection(mode, model=model, **kwargs)

    def disable_detection(self):
        """Disable the vision detection pipeline. Delegates to: vision.disable_detection()"""
        return self.vision.disable_detection()

    def get_active_mode(self):
        """Get the currently active vision mode. Delegates to: vision.get_active_mode()"""
        return self.vision.get_active_mode()

    def get_detections(self, class_filter=None):
        """Get latest detection results. Delegates to: vision.get_detections()"""
        return self.vision.get_detections(class_filter=class_filter)

    def get_faces(self):
        """Get latest face detection results. Delegates to: vision.get_faces()"""
        return self.vision.get_faces()

    def get_pose_keypoints(self):
        """Get latest pose keypoints. Delegates to: vision.get_pose_keypoints()"""
        return self.vision.get_pose_keypoints()

    def wait_for_detection(self, target_class, timeout=5.0):
        """Wait for a specific detection class. Delegates to: vision.wait_for_detection()"""
        return self.vision.wait_for_detection(target_class, timeout=timeout)

    def get_gesture(self):
        """Get current gesture class. Delegates to: vision.get_gesture()"""
        return self.vision.get_gesture()

    def get_gesture_full(self):
        """Get full gesture result dict. Delegates to: vision.get_gesture_full()"""
        return self.vision.get_gesture_full()

    def get_aruco_markers(self):
        """Get list of ArUco markers. Delegates to: vision.get_aruco_markers()"""
        return self.vision.get_aruco_markers()

    def wait_for_marker(self, marker_id, timeout=5.0):
        """Wait for specific ArUco marker. Delegates to: vision.wait_for_marker()"""
        return self.vision.wait_for_marker(marker_id, timeout=timeout)

    def wait_for_gesture(self, gesture_name, timeout=5.0):
        """Wait for a specific gesture. Delegates to: vision.wait_for_gesture()"""
        return self.vision.wait_for_gesture(gesture_name, timeout=timeout)

    def wait_for_face(self, timeout=5.0):
        """Wait for any face to be detected. Delegates to: vision.wait_for_face()"""
        return self.vision.wait_for_face(timeout=timeout)

    def wait_for_pose(self, timeout=5.0):
        """Wait for pose keypoints to be detected. Delegates to: vision.wait_for_pose()"""
        return self.vision.wait_for_pose(timeout=timeout)

    # ═══════════════════════════════════════════════════════════════════════
