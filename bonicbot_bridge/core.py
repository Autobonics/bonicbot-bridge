"""
Core BonicBot class for robot control
"""

import time
import math
from roslibpy import Ros, Topic, Service, ServiceRequest
from .motion import MotionController
from .sensors import SensorManager  
from .system import SystemController
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