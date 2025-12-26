"""
Sensor manager for accessing robot sensor data
"""

import time
from roslibpy import Topic
from .exceptions import BonicBotError

class SensorManager:
    def __init__(self, ros_client):
        self.ros = ros_client
        
        # Current sensor data
        self.current_pose = None
        self.battery_level = 0.0
        self.lidar_data = None
        
        # Subscribers
        self.odom_sub = Topic(self.ros, '/diff_cont/odom', 'nav_msgs/Odometry') 
        
        # Start subscriptions
        self.odom_sub.subscribe(self._odom_callback)
        
        # Wait a moment for initial data
        time.sleep(0.5)
    
    def _odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_pose = msg['pose']['pose']
    
    def get_position(self):
        """
        Get current robot position
        
        Returns:
            dict: {'x': float, 'y': float, 'theta': float (degrees)} or None if no data
        """
        if not self.current_pose:
            return None
            
        pos = self.current_pose['position']
        orientation = self.current_pose['orientation']
        
        # Convert quaternion to yaw angle (in radians first)
        import math
        theta_rad = 2 * math.atan2(orientation['z'], orientation['w'])
        # Convert to degrees
        theta_deg = math.degrees(theta_rad)
        
        return {
            'x': pos['x'],
            'y': pos['y'], 
            'theta': theta_deg
        }
    
    def get_x(self):
        """Get current X position in meters"""
        pos = self.get_position()
        return pos['x'] if pos else 0.0
    
    def get_y(self):
        """Get current Y position in meters"""
        pos = self.get_position()
        return pos['y'] if pos else 0.0
        
    def get_heading(self):
        """Get current robot heading in degrees"""
        pos = self.get_position()
        return pos['theta'] if pos else 0.0
        
    def get_heading_degrees(self):
        """Get current robot heading in degrees (alias for get_heading)"""
        return self.get_heading()
    
    def get_battery(self):
        """
        Get battery level percentage (0-100)
        Note: Implement based on your robot's battery topic
        """
        # TODO: Subscribe to actual battery topic when available
        # For now return a placeholder
        return 85.0
    
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
            return 0.0
            
        current = self.get_position()
        if not current:
            return 0.0
            
        import math
        dx = current['x'] - start_pos['x']
        dy = current['y'] - start_pos['y']
        return math.sqrt(dx*dx + dy*dy)
    
    def wait_for_data(self, timeout=5):
        """
        Wait for sensor data to become available
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            bool: True if data received, False on timeout
        """
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if self.current_pose:
                return True
            time.sleep(0.1)
            
        return False
    
    def subscribe_to_position(self, callback):
        """
        Subscribe to position updates
        
        Args:
            callback: Function to call with position data
                     callback(x, y, theta)
        """
        def wrapper(msg):
            pos = self.get_position()
            if pos:
                callback(pos['x'], pos['y'], pos['theta'])
        
        self.odom_sub.subscribe(lambda msg: wrapper(msg))
    
    def get_sensor_info(self):
        """
        Get summary of all available sensor data
        
        Returns:
            dict: Summary of sensor states
        """
        pos = self.get_position()
        
        return {
            'position': pos,
            'battery': self.get_battery(),
            'sensors_active': pos is not None,
            'timestamp': time.time()
        }