"""
Motion controller for robot movement and navigation
"""

import time
import math
from roslibpy import Topic, Service, ServiceRequest
from .exceptions import NavigationError

class MotionController:
    def __init__(self, ros_client):
        self.ros = ros_client
        
        # Movement publisher
        self.cmd_vel_pub = Topic(self.ros, '/cmd_vel', 'geometry_msgs/Twist')
        
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
        
    def _nav_status_callback(self, msg):
        """Update navigation status"""
        self.nav_status = msg['data']
        
    def _distance_callback(self, msg):
        """Update distance to goal"""
        self.distance_to_goal = msg['data']
    
    def move(self, linear_x=0, linear_y=0, angular_z=0):
        """
        Send velocity command to robot
        
        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s) - for omnidirectional robots
            angular_z: Rotational velocity (deg/s)
        """
        # Convert angular velocity from deg/s to rad/s for ROS
        angular_z_rad = math.radians(angular_z)
        
        msg = {
            'linear': {'x': linear_x, 'y': linear_y, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z_rad}
        }
        self.cmd_vel_pub.publish(msg)
    
    def move_forward(self, speed=0.3, duration=None):
        """
        Move robot forward
        
        Args:
            speed: Forward speed in m/s (default: 0.3)
            duration: Time to move in seconds (None for continuous)
        """
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
    
    def move_backward(self, speed=0.3, duration=None):
        """Move robot backward"""
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
            
    def turn_left(self, speed=0.5, duration=None):
        """Turn robot left (counter-clockwise)"""
        if duration:
            # Continuously publish commands to avoid cmd_vel_timeout
            publish_rate = 10  # 10 Hz
            interval = 1.0 / publish_rate
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                self.move(angular_z=speed)
                time.sleep(interval)
            
            self.stop()
        else:
            # Continuous movement (single command)
            self.move(angular_z=speed)
            
    def turn_right(self, speed=0.5, duration=None):
        """Turn robot right (clockwise)"""
        if duration:
            # Continuously publish commands to avoid cmd_vel_timeout
            publish_rate = 10  # 10 Hz
            interval = 1.0 / publish_rate
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                self.move(angular_z=-speed)
                time.sleep(interval)
            
            self.stop()
        else:
            # Continuous movement (single command)
            self.move(angular_z=-speed)
    
    def stop(self):
        """Stop all robot movement"""
        self.move(0, 0, 0)
    
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