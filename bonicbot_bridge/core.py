"""
Core BonicBot class for robot control
"""

import time
import math
from roslibpy import Ros, Topic, Service, ServiceRequest
from .motion import MotionController
from .sensors import SensorManager  
from .system import SystemController
from .camera import CameraManager
from .servo import ServoController
from .exceptions import ConnectionError, BonicBotError

class BonicBot:
    def __init__(self, host='localhost', port=9090, timeout=10):
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
        self.camera = None
        self.servo = None
        
        # Connect to robot
        self.connect(timeout)
        
    def connect(self, timeout=10):
        """Establish connection to robot"""
        try:
            self.ros = Ros(host=self.host, port=self.port)
            self.ros.run()
            
            # Wait for connection
            start_time = time.time()
            while not self.ros.is_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
                
            if not self.ros.is_connected:
                raise ConnectionError(f"Failed to connect to robot at {self.host}:{self.port}")
                
            # Initialize controllers
            self.motion = MotionController(self.ros)
            self.sensors = SensorManager(self.ros)
            self.system = SystemController(self.ros)
            self.camera = CameraManager(self.ros)
            self.servo = ServoController(self.ros)
            
            self.connected = True
            print(f"🤖 Connected to BonicBot at {self.host}:{self.port}")
            
        except Exception as e:
            raise ConnectionError(f"Connection failed: {str(e)}")
    
    def disconnect(self):
        """Disconnect from robot"""
        if self.ros and self.ros.is_connected:
            self.motion.stop()  # Safety stop
            self.ros.terminate()
            self.connected = False
            print("🔌 Disconnected from BonicBot")
    
    def __enter__(self):
        """Context manager entry"""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()
    
    # Quick access methods (delegate to controllers)
    def move_forward(self, speed=0.3, duration=None):
        """Move robot forward"""
        return self.motion.move_forward(speed, duration)
    
    def move_backward(self, speed=0.3, duration=None):
        """Move robot backward"""
        return self.motion.move_backward(speed, duration)
        
    def turn_left(self, speed=0.5, duration=None):
        """Turn robot left"""
        return self.motion.turn_left(speed, duration)
        
    def turn_right(self, speed=0.5, duration=None):
        """Turn robot right"""
        return self.motion.turn_right(speed, duration)
    
    def stop(self):
        """Stop robot movement"""
        return self.motion.stop()
        
    def go_to(self, x, y, theta=0):
        """Navigate to specific coordinate"""
        return self.motion.go_to(x, y, theta)
        
    def get_battery(self):
        """Get battery percentage"""
        return self.sensors.get_battery()
        
    def get_position(self):
        """Get current robot position"""
        return self.sensors.get_position()
        
    def start_mapping(self):
        """Start mapping mode"""
        return self.system.start_mapping()
        
    def stop_mapping(self):
        """Stop mapping mode"""  
        return self.system.stop_mapping()
        
    def save_map(self):
        """Save current map"""
        return self.system.save_map()
    
    def is_connected(self):
        """Check if connected to robot"""
        return self.connected and self.ros and self.ros.is_connected

    def wait_for_goal(self, timeout=30):
        """Wait for current navigation goal to complete"""
        return self.motion.wait_for_goal(timeout)
    
    def get_distance_to_goal(self):
        """Get distance to current navigation goal"""
        return self.motion.get_distance_to_goal()
    
    # Camera shortcuts
    def start_camera(self, callback=None):
        """Start camera streaming"""
        return self.camera.start_streaming(callback=callback)
    
    def stop_camera(self):
        """Stop camera streaming"""
        return self.camera.stop_streaming()
    
    def get_image(self):
        """Get latest camera image"""
        return self.camera.get_latest_image()
    
    def save_image(self, filepath):
        """Save current camera image"""
        return self.camera.save_image(filepath)
    
    # Servo shortcuts
    def set_servos(self, angles):
        """Set servo angles (dictionary of joint_name: angle_degrees)"""
        return self.servo.set_servo_angles(angles)
    
    def move_left_arm(self, shoulder, elbow):
        """Move left arm (shoulder, elbow angles in degrees)"""
        return self.servo.move_left_arm(shoulder, elbow)
    
    def move_right_arm(self, shoulder, elbow):
        """Move right arm (shoulder, elbow angles in degrees)"""
        return self.servo.move_right_arm(shoulder, elbow)
    
    def set_grippers(self, left, right):
        """Set gripper angles in degrees"""
        return self.servo.set_grippers(left, right)
    
    def open_grippers(self):
        """Open both grippers"""
        return self.servo.open_grippers()
    
    def close_grippers(self):
        """Close both grippers"""
        return self.servo.close_grippers()
    
    def set_neck(self, yaw):
        """Set neck yaw angle in degrees"""
        return self.servo.set_neck(yaw)
    
    def look_left(self):
        """Turn neck fully left"""
        return self.servo.look_left()
    
    def look_right(self):
        """Turn neck fully right"""
        return self.servo.look_right()
    
    def look_center(self):
        """Center the neck"""
        return self.servo.look_center()
    
    def reset_servos(self):
        """Reset all servos to neutral position"""
        return self.servo.reset_all_servos()