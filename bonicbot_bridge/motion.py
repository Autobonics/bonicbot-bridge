"""
Motion controller for robot movement and navigation
"""

import time
import math
import threading
import functools
from roslibpy import Topic, Service, ServiceRequest
from .exceptions import NavigationError
from .precisemotion import QueueMixin

# ── Velocity safety limits ─────────────────────────────────────────────
# Conservative defaults for a small differential-drive indoor robot.
# These can be overridden at runtime via MotionController.set_speed_limits().
MAX_LINEAR_SPEED = 1.0       # m/s  (absolute value, applies to linear_x & linear_y)
MAX_ANGULAR_SPEED = 180.0    # deg/s (absolute value, applied before rad/s conversion)

def _with_motion_lock(func):
    """Decorator to ensure motion commands are thread-safe and don't interleave."""
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        with self._motion_lock:
            return func(self, *args, **kwargs)
    return wrapper

class MotionController(QueueMixin):
    def __init__(self, ros_client):
        self.ros = ros_client
        
        # Velocity limits (instance-level so they can be tuned per robot)
        self._max_linear_speed = MAX_LINEAR_SPEED
        self._max_angular_speed = MAX_ANGULAR_SPEED
        
        # Thread safety lock for motion commands
        self._motion_lock = threading.RLock()
        
        # Movement publisher
        self.cmd_vel_pub = Topic(self.ros, '/cmd_vel', 'geometry_msgs/Twist')
        self.cmd_vel_pub.advertise()
        
        self._init_queue()
        
        # Odometry subscription for closed-loop control
        self._odom_sub = Topic(self.ros, '/diff_cont/odom', 'nav_msgs/Odometry')
        self._current_yaw = None  # radians
        self._yaw_lock = threading.Lock()
        self._odom_sub.subscribe(self._odom_callback)
        
        # Navigation topics and services
        self.goal_pub = Topic(self.ros, '/goal_pose', 'geometry_msgs/PoseStamped')
        self.nav_status_sub = Topic(self.ros, '/robot/nav_status', 'std_msgs/String')
        self.distance_sub = Topic(self.ros, '/robot/distance_to_goal', 'std_msgs/Float32')
        
        # Navigation services
        self.start_nav_srv = Service(self.ros, '/robot/start_navigation', 'std_srvs/Trigger')
        self.stop_nav_srv = Service(self.ros, '/robot/stop_navigation', 'std_srvs/Trigger') 
        self.cancel_nav_srv = Service(self.ros, '/robot/cancel_navigation', 'std_srvs/Trigger')
        
        # State tracking
        self.nav_status = 'idle'
        self.distance_to_goal = 0.0
        
        # Subscribe to status updates
        self.nav_status_sub.subscribe(self._nav_status_callback)
        self.distance_sub.subscribe(self._distance_callback)
        
        # Navigation active tracking
        self.nav_active_sub = Topic(self.ros, '/robot/navigation_active', 'std_msgs/Bool')
        self.navigation_active = False
        self.nav_active_sub.subscribe(self._nav_active_callback)
        
    def _odom_callback(self, msg):
        """Extract yaw from odometry quaternion for closed-loop turns."""
        q = msg['pose']['pose']['orientation']
        yaw = 2.0 * math.atan2(q['z'], q['w'])
        with self._yaw_lock:
            self._current_yaw = yaw

    def _nav_status_callback(self, msg):
        """Update navigation status"""
        self.nav_status = msg['data']
        
    def _distance_callback(self, msg):
        """Update distance to goal"""
        self.distance_to_goal = msg['data']
        
    def _nav_active_callback(self, msg):
        """Update navigation active status"""
        self.navigation_active = msg['data']
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def _wait_for_yaw(self, timeout=5.0):
        """Block until odometry yaw is available."""
        start = time.time()
        while time.time() - start < timeout:
            with self._yaw_lock:
                if self._current_yaw is not None:
                    return self._current_yaw
            time.sleep(0.05)
        return None
    
    @_with_motion_lock
    def _turn_by_angle(self, angle_deg, speed_deg, timeout=30.0):
        """
        Closed-loop turn using odometry feedback.

        Args:
            angle_deg (float): How many degrees to turn.
                               Positive = counter-clockwise (left).
                               Negative = clockwise (right).
            speed_deg (float): Rotation speed in deg/s (always positive).
            timeout (float):   Safety timeout in seconds.

        Returns:
            float: Actual angle turned in degrees.
        """
        angle_rad = math.radians(angle_deg)
        speed_rad = math.radians(speed_deg)

        # Determine angular velocity direction
        omega = speed_rad if angle_rad > 0 else -speed_rad

        # Wait for initial yaw reading
        start_yaw = self._wait_for_yaw()
        if start_yaw is None:
            # Fallback to open-loop if no odometry available
            fallback_duration = abs(angle_deg) / speed_deg
            self._open_loop_turn(omega, fallback_duration)
            return angle_deg  # nominal, unverified

        target_delta = abs(angle_rad)
        accumulated = 0.0
        prev_yaw = start_yaw

        publish_rate = 20  # 20 Hz for smooth control
        interval = 1.0 / publish_rate
        start_time = time.time()

        while accumulated < target_delta:
            if (time.time() - start_time) > timeout:
                break

            self.move(angular_z=math.degrees(omega))
            time.sleep(interval)

            with self._yaw_lock:
                current_yaw = self._current_yaw

            if current_yaw is not None:
                delta = self._normalize_angle(current_yaw - prev_yaw)
                accumulated += abs(delta)
                prev_yaw = current_yaw

        self.stop()
        return math.degrees(accumulated) * (1.0 if angle_rad > 0 else -1.0)
    
    # ── Speed limit helpers ───────────────────────────────────────────

    def set_speed_limits(self, max_linear=None, max_angular=None):
        """Override the default velocity safety limits.

        Call this after construction if the robot's hardware supports
        different speed ranges than the built-in defaults.

        Args:
            max_linear (float | None): Maximum absolute linear speed in m/s.
                If ``None``, the current limit is kept.
            max_angular (float | None): Maximum absolute angular speed in deg/s.
                If ``None``, the current limit is kept.

        Raises:
            ValueError: If a supplied limit is not positive.
        """
        if max_linear is not None:
            if max_linear <= 0:
                raise ValueError("max_linear must be positive")
            self._max_linear_speed = float(max_linear)
        if max_angular is not None:
            if max_angular <= 0:
                raise ValueError("max_angular must be positive")
            self._max_angular_speed = float(max_angular)

    def get_speed_limits(self):
        """Return the current velocity safety limits.

        Returns:
            dict: ``{'max_linear': float, 'max_angular': float}``
                  Linear in m/s, angular in deg/s.
        """
        return {
            'max_linear': self._max_linear_speed,
            'max_angular': self._max_angular_speed,
        }

    def _clamp_velocity(self, value, limit, label):
        """Clamp *value* to [-limit, +limit] and warn if clamped.

        Follows the same clamp-and-warn pattern as
        ``ServoController._validate_angle`` — the command still executes,
        but at the safe maximum instead of the dangerous requested value.

        Args:
            value (float): Requested velocity component.
            limit (float): Maximum absolute value.
            label (str): Human-readable name for the warning message.

        Returns:
            float: Clamped velocity.
        """
        if abs(value) > limit:
            clamped = max(-limit, min(limit, value))
            print(
                f"⚠️ {label}={value} exceeds limit ±{limit}, "
                f"clamping to {clamped}"
            )
            return clamped
        return value

    @_with_motion_lock
    def move(self, linear_x=0, linear_y=0, angular_z=0):
        """
        Send velocity command to robot.

        Values are clamped to the configured safety limits before
        publishing.  Use ``set_speed_limits()`` to adjust the limits
        for your specific hardware.

        Args:
            linear_x (float): Forward/backward velocity in m/s.
                              Positive = forward, negative = backward.
            linear_y (float): Left/right strafe velocity in m/s.
                              Only effective on omnidirectional robots.
            angular_z (float): Rotational velocity in **degrees/second** (deg/s).
                               Positive = counter-clockwise (left), negative = clockwise (right).
                               This value is converted to rad/s internally before being sent
                               to the ROS cmd_vel topic.

        Note:
            Due to ROS2's cmd_vel_timeout (~0.5s), a single call only moves the robot
            briefly. For sustained movement, publish in a 10 Hz loop or use the
            higher-level convenience methods (move_forward, turn_left, etc.).

        Examples:
            >>> bot.motion.move(linear_x=0.3)          # Forward at 0.3 m/s
            >>> bot.motion.move(angular_z=30.0)        # Spin left at 30 deg/s
            >>> bot.motion.move(linear_x=0.2, angular_z=-20.0)  # Arc right
        """
        # Clamp velocities to configured safety limits
        linear_x = self._clamp_velocity(
            linear_x, self._max_linear_speed, 'linear_x'
        )
        linear_y = self._clamp_velocity(
            linear_y, self._max_linear_speed, 'linear_y'
        )
        angular_z = self._clamp_velocity(
            angular_z, self._max_angular_speed, 'angular_z'
        )

        # Convert angular velocity from deg/s to rad/s for ROS
        angular_z_rad = math.radians(angular_z)
        
        msg = {
            'linear': {'x': linear_x, 'y': linear_y, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z_rad}
        }
        self.cmd_vel_pub.publish(msg)
    
    @_with_motion_lock
    def move_forward(self, speed=0.3, duration=None):
        """
        Move robot forward
        
        Args:
            speed: Forward speed in m/s (default: 0.3)
            duration: Time to move in seconds (None for continuous)
        """
        if duration is not None and duration < 0:
            raise ValueError("Duration cannot be negative")
            
        if duration:
            # Continuously publish commands to avoid cmd_vel_timeout
            publish_rate = 10  # 10 Hz
            interval = 1.0 / publish_rate
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                self.move(linear_x=speed)
                time.sleep(interval)
            
            self.stop()
        else:
            # Continuous movement (single command)
            self.move(linear_x=speed)
    
    @_with_motion_lock
    def move_backward(self, speed=0.3, duration=None):
        """Move robot backward"""
        if duration is not None and duration < 0:
            raise ValueError("Duration cannot be negative")
            
        if duration:
            # Continuously publish commands to avoid cmd_vel_timeout
            publish_rate = 10  # 10 Hz
            interval = 1.0 / publish_rate
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                self.move(linear_x=-speed)
                time.sleep(interval)
            
            self.stop()
        else:
            # Continuous movement (single command)
            self.move(linear_x=-speed)
            
    @_with_motion_lock
    def _open_loop_turn(self, omega_rad, duration):
        """Fallback open-loop timed turn (used when odometry is unavailable)."""
        publish_rate = 20  # Hz
        interval = 1.0 / publish_rate
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.move(angular_z=math.degrees(omega_rad))
            time.sleep(interval)
        self.stop()

    @_with_motion_lock
    def turn_left(self, speed=30.0, angle=None, duration=None):
        """
        Turn robot left (counter-clockwise).

        Uses **closed-loop odometry feedback** when ``angle`` is specified,
        guaranteeing accurate rotation regardless of execution environment.
        Falls back to open-loop timed publishing when ``duration`` is used.

        Args:
            speed (float): Rotational speed in **degrees/second** (deg/s).
                           Default is 30 deg/s.
            angle (float | None): Target rotation in **degrees**.
                                  Uses odometry feedback for precise control.
                                  Example: angle=90 → exact 90° left turn.
            duration (float | None): **(Legacy / fallback)** Time to turn in
                                     seconds. Ignored if ``angle`` is provided.
                                     If both are None, sends a single command
                                     (continuous until stopped).

        Returns:
            float | None: Actual degrees turned (when ``angle`` is used),
                          or None for continuous / duration modes.

        Examples:
            >>> bot.turn_left(angle=90)             # Precise 90° left turn
            >>> bot.turn_left(speed=60, angle=180)  # Fast 180° left turn
            >>> bot.turn_left(speed=30, duration=3)  # Legacy open-loop mode
            >>> bot.turn_left(speed=30)              # Continuous spin
        """
        if duration is not None and duration < 0:
            raise ValueError("Duration cannot be negative")
            
        if angle is not None:
            return self._turn_by_angle(angle_deg=abs(angle), speed_deg=abs(speed))
        elif duration:
            self._open_loop_turn(math.radians(speed), duration)
        else:
            self.move(angular_z=speed)
            
    @_with_motion_lock
    def turn_right(self, speed=30.0, angle=None, duration=None):
        """
        Turn robot right (clockwise).

        Uses **closed-loop odometry feedback** when ``angle`` is specified,
        guaranteeing accurate rotation regardless of execution environment.
        Falls back to open-loop timed publishing when ``duration`` is used.

        Args:
            speed (float): Rotational speed in **degrees/second** (deg/s).
                           Default is 30 deg/s.
            angle (float | None): Target rotation in **degrees**.
                                  Uses odometry feedback for precise control.
                                  Example: angle=90 → exact 90° right turn.
            duration (float | None): **(Legacy / fallback)** Time to turn in
                                     seconds. Ignored if ``angle`` is provided.
                                     If both are None, sends a single command
                                     (continuous until stopped).

        Returns:
            float | None: Actual degrees turned (when ``angle`` is used),
                          or None for continuous / duration modes.

        Examples:
            >>> bot.turn_right(angle=90)             # Precise 90° right turn
            >>> bot.turn_right(speed=60, angle=180)  # Fast 180° right turn
            >>> bot.turn_right(speed=30, duration=3)  # Legacy open-loop mode
            >>> bot.turn_right(speed=30)              # Continuous spin
        """
        if duration is not None and duration < 0:
            raise ValueError("Duration cannot be negative")
            
        if angle is not None:
            return self._turn_by_angle(angle_deg=-abs(angle), speed_deg=abs(speed))
        elif duration:
            self._open_loop_turn(-math.radians(speed), duration)
        else:
            self.move(angular_z=-speed)
    
    @_with_motion_lock
    def stop(self):
        """Stop all robot movement"""
        self.move(0, 0, 0)
        
    def _validate_coordinate(self, value, name, limit=10000.0):
        """Validate coordinate is finite and within reasonable bounds."""
        try:
            val = float(value)
        except (TypeError, ValueError):
            raise NavigationError(f"Coordinate '{name}' must be a numeric value.")
            
        if not math.isfinite(val):
            raise NavigationError(f"Coordinate '{name}' cannot be NaN or infinity.")
            
        if limit is not None and abs(val) > limit:
            raise NavigationError(f"Coordinate '{name}' exceeds maximum allowed range (±{limit}).")
            
        return val
    
    def go_to(self, x, y, theta=0):
        """
        Navigate to specific coordinate using Nav2
        
        Args:
            x: Target X coordinate (meters)
            y: Target Y coordinate (meters) 
            theta: Target orientation (degrees, default: 0)
            
        Returns:
            bool: True if goal was sent successfully
        """
        if not self.navigation_active:
            raise NavigationError("Cannot set goal: Navigation system is not active. Call start_navigation() first.")
            
        x = self._validate_coordinate(x, 'x')
        y = self._validate_coordinate(y, 'y')
        theta = self._validate_coordinate(theta, 'theta', limit=None)
            
        try:
            # Convert degrees to radians for ROS message
            theta_rad = math.radians(theta)
            
            # Create goal message
            goal_msg = {
                'header': {
                    'stamp': {'sec': 0, 'nanosec': 0},
                    'frame_id': 'map'
                },
                'pose': {
                    'position': {'x': x, 'y': y, 'z': 0.0},
                    'orientation': {
                        'x': 0.0, 'y': 0.0, 
                        'z': math.sin(theta_rad/2), 
                        'w': math.cos(theta_rad/2)
                    }
                }
            }
            
            # Publish goal
            self.goal_pub.publish(goal_msg)
            print(f"🎯 Navigation goal set: ({x:.2f}, {y:.2f}, θ={theta:.1f}°)")
            return True
            
        except Exception as e:
            raise NavigationError(f"Failed to set navigation goal: {str(e)}")
    
    def start_navigation(self):
        """Start navigation system"""
        request = ServiceRequest()
        response = self.start_nav_srv.call(request)
        
        if not response['success']:
            raise NavigationError(f"Failed to start navigation: {response['message']}")
        
        print("🧭 Navigation system started")
        return True
    
    def stop_navigation(self):
        """Stop navigation system"""
        request = ServiceRequest()
        response = self.stop_nav_srv.call(request)
        
        if not response['success']:
            raise NavigationError(f"Failed to stop navigation: {response['message']}")
        
        print("🛑 Navigation system stopped") 
        return True
    
    def cancel_goal(self):
        """Cancel current navigation goal"""
        request = ServiceRequest()
        response = self.cancel_nav_srv.call(request)
        
        if not response['success']:
            raise NavigationError(f"Failed to cancel goal: {response['message']}")
            
        print("❌ Navigation goal cancelled")
        return True
    
    def set_initial_pose(self, x, y, theta=0):
        """
        Set initial pose for robot localization
        
        Args:
            x: Initial X coordinate (meters)
            y: Initial Y coordinate (meters)
            theta: Initial orientation (degrees, default: 0)
            
        Returns:
            bool: True if pose was set successfully
        """
        x = self._validate_coordinate(x, 'x')
        y = self._validate_coordinate(y, 'y')
        theta = self._validate_coordinate(theta, 'theta', limit=None)
        
        try:
            # Convert degrees to radians for ROS message
            theta_rad = math.radians(theta)
            
            # Create initial pose topic
            initial_pose_pub = Topic(
                self.ros,
                '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped'
            )
            
            initial_pose_pub.advertise()
            time.sleep(0.15)  # Wait for topic to be ready
            
            # Create pose message
            pose_msg = {
                'header': {
                    'stamp': {
                        'sec': int(time.time()),
                        'nanosec': int((time.time() % 1) * 1e9)
                    },
                    'frame_id': 'map'
                },
                'pose': {
                    'pose': {
                        'position': {'x': x, 'y': y, 'z': 0.0},
                        'orientation': {
                            'x': 0.0,
                            'y': 0.0,
                            'z': math.sin(theta_rad / 2),
                            'w': math.cos(theta_rad / 2)
                        }
                    },
                    'covariance': [0.0] * 36  # 6x6 covariance matrix
                }
            }
            
            # Publish initial pose
            initial_pose_pub.publish(pose_msg)
            print(f"📍 Initial pose set: ({x:.2f}, {y:.2f}, θ={theta:.1f}°)")
            
            time.sleep(0.2)
            initial_pose_pub.unadvertise()
            
            return True
            
        except Exception as e:
            raise NavigationError(f"Failed to set initial pose: {str(e)}")
    
    def wait_for_goal(self, timeout=30):
        """
        Wait for current navigation goal to complete
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            str: Final navigation status ('goal_reached', 'goal_failed', 'cancelled')
        """
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if self.nav_status in ['goal_reached', 'goal_failed', 'cancelled']:
                if self.nav_status == 'goal_reached':
                    print("✅ Goal reached!")
                elif self.nav_status == 'goal_failed':
                    print("❌ Goal failed!")
                else:
                    print("🚫 Goal cancelled!")
                return self.nav_status
                
            time.sleep(0.1)
        
        print(f"⏰ Navigation timeout after {timeout}s")
        return 'timeout'
    
    def get_nav_status(self):
        """Get current navigation status"""
        return self.nav_status
        
    def get_distance_to_goal(self):
        """Get distance to current navigation goal in meters"""
        return self.distance_to_goal
    
    def is_moving(self):
        """Check if robot is currently moving"""
        return self.nav_status == 'navigating'
    def shutdown(self):
        """Release motion subscriptions during teardown."""
        try:
            self._queue_shutdown()
        except Exception as exc:
            pass
        
        try:
            self.stop()
        except Exception:
            pass
            
        for pub in (self.goal_pub, self.cmd_vel_pub):
            try:
                pub.unadvertise()
            except Exception:
                pass
                
        for sub in (self._odom_sub, self.nav_status_sub, self.distance_sub, self.nav_active_sub):
            try:
                sub.unsubscribe()
            except Exception:
                pass
