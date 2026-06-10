"""
Core BonicBot class for robot control
"""

import time
import math
import threading
from roslibpy import Ros, Topic, Service, ServiceRequest
from .motion import MotionController
from .sensors import SensorManager  
from .system import SystemController
from .vision import VisionController
from .autonomous import ExploreController
from .exceptions import BonicConnectionError, BonicBotError

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
        self._reconnect_lock = threading.Lock()
        self._reconnecting = False
        
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
                raise BonicConnectionError(f"Failed to connect to robot at {self.host}:{self.port}")
                
            # Initialize controllers
            self.motion = MotionController(self.ros)
            self.sensors = SensorManager(self.ros)
            self.system = SystemController(self.ros)
            self.camera = CameraManager(self.ros)
            self.servo = ServoController(self.ros)
            self.vision = VisionController(self.ros)
            self.autonomous = ExploreController(self.ros)
            
            self.connected = True
            self.ros.on('close', lambda event: self._on_connection_lost())
            print(f"🤖 Connected to BonicBot at {self.host}:{self.port}")
            
        except Exception as e:
            raise BonicConnectionError(f"Connection failed: {str(e)}")
    
    def disconnect(self):
        """Disconnect from robot"""
        self.connected = False  # Set first to prevent reconnect handler
        if self.ros and self.ros.is_connected:
            try:
                self.motion.stop()  # Safety stop
            except Exception:
                pass
            self.ros.close()
            # Give websocket time to close cleanly
            time.sleep(0.5)
            print("🔌 Disconnected from BonicBot")
    
    def _on_connection_lost(self):
        """Handle unexpected websocket disconnection."""
        if not self.connected:
            return  # Intentional disconnect, don't reconnect
        self.connected = False
        print("⚠️ WebSocket connection lost — attempting reconnection...")
        threading.Thread(target=self._reconnect_loop, daemon=True).start()
    
    def _reconnect_loop(self, max_attempts=5, base_delay=2.0):
        """Reconnect with exponential backoff."""
        with self._reconnect_lock:
            if self._reconnecting:
                return
            self._reconnecting = True
        
        try:
            for attempt in range(1, max_attempts + 1):
                delay = min(base_delay * (2 ** (attempt - 1)), 30.0)
                print(f"  🔄 Reconnect attempt {attempt}/{max_attempts} in {delay:.1f}s...")
                time.sleep(delay)
                
                try:
                    try:
                        self.ros.close()
                    except Exception:
                        pass
                    
                    self.ros = Ros(host=self.host, port=self.port)
                    self.ros.run()
                    
                    t0 = time.time()
                    while not self.ros.is_connected and (time.time() - t0) < 10:
                        time.sleep(0.1)
                    
                    if self.ros.is_connected:
                        self._reinitialize_controllers()
                        self.ros.on('close', lambda event: self._on_connection_lost())
                        self.connected = True
                        print(f"  ✅ Reconnected on attempt {attempt}")
                        return
                except Exception as e:
                    print(f"  ❌ Attempt {attempt} failed: {e}")
            
            print("  ❌ All reconnection attempts exhausted")
        finally:
            self._reconnecting = False
    
    def _reinitialize_controllers(self):
        """Re-create all controllers with the new connection."""
        try:
            self.motion = MotionController(self.ros)
            self.sensors = SensorManager(self.ros)
            self.system = SystemController(self.ros)
            self.camera = CameraManager(self.ros)
            self.servo = ServoController(self.ros)
            self.vision = VisionController(self.ros)
            self.autonomous = ExploreController(self.ros)
        except Exception as e:
            print(f"  ⚠️ Controller re-init partial failure: {e}")
    
    def __enter__(self):
        """Context manager entry"""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()
    
    # Quick access methods (delegate to controllers)
    def move(self, linear_x=0, linear_y=0, angular_z=0):
        """Low-level velocity control for custom robot movement patterns"""
        return self.motion.move(linear_x, linear_y, angular_z)
        
    def move_forward(self, speed=0.3, duration=None):
        """Move robot forward"""
        return self.motion.move_forward(speed, duration)
    
    def move_backward(self, speed=0.3, duration=None):
        """Move robot backward"""
        return self.motion.move_backward(speed, duration)
        
    def turn_left(self, speed=30.0, angle=None, duration=None):
        """Turn robot left (closed-loop when angle is specified)"""
        return self.motion.turn_left(speed, angle=angle, duration=duration)
        
    def turn_right(self, speed=30.0, angle=None, duration=None):
        """Turn robot right (closed-loop when angle is specified)"""
        return self.motion.turn_right(speed, angle=angle, duration=duration)
    
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
    
    def get_x(self):
        """Get current X position in meters"""
        return self.sensors.get_x()
    
    def get_y(self):
        """Get current Y position in meters"""
        return self.sensors.get_y()
    
    def get_heading(self):
        """Get current robot heading in degrees"""
        return self.sensors.get_heading()
        
    def get_heading_degrees(self):
        """Get current robot heading in degrees"""
        return self.sensors.get_heading_degrees()
        
    def start_mapping(self):
        """Start mapping mode"""
        return self.system.start_mapping()
        
    def stop_mapping(self):
        """Stop mapping mode"""  
        return self.system.stop_mapping()
        
    def save_map(self):
        """Save current map"""
        return self.system.save_map()
    
    def start_navigation(self):
        """Start navigation system"""
        return self.motion.start_navigation()
    
    def stop_navigation(self):
        """Stop navigation system"""
        return self.motion.stop_navigation()
    
    def cancel_goal(self):
        """Cancel current navigation goal"""
        return self.motion.cancel_goal()
    
    def get_nav_status(self):
        """Get current navigation status"""
        return self.motion.get_nav_status()
    
    def is_moving(self):
        """Check if robot is currently moving"""
        return self.motion.is_moving()
    
    def get_system_status(self):
        """Get system status information"""
        return self.system.get_system_status()
    
    def is_mapping(self):
        """Check if robot is currently mapping"""
        return self.system.is_mapping()
    
    def is_navigating(self):
        """Check if navigation system is active"""
        return self.system.is_navigating()
    
    def is_connected(self):
        """Check if connected to robot"""
        return self.connected and self.ros and self.ros.is_connected

    def wait_for_goal(self, timeout=30):
        """Wait for current navigation goal to complete"""
        return self.motion.wait_for_goal(timeout)
    
    def get_distance_to_goal(self):
        """Get distance to current navigation goal"""
        return self.motion.get_distance_to_goal()
    
    def set_initial_pose(self, x, y, theta=0):
        """Set initial pose for localization"""
        return self.motion.set_initial_pose(x, y, theta)
    
    # Camera methods
    def start_camera(self, callback=None):
        """
        Start camera streaming (client-side subscription)
        
        Note: Call camera.start_camera_service() first to activate robot's camera hardware,
        then call this to start receiving images in your script.
        
        Args:
            callback: Optional function(image) called for each frame
        """
        return self.camera.start_streaming(callback=callback)
    
    def stop_camera(self):
        """
        Stop camera streaming (client-side subscription)
        
        Note: Call camera.stop_camera_service() after this to deactivate robot's camera
        hardware for better performance.
        """
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
    
    def wave_left_arm(self, duration=2.0):
        """Wave left arm"""
        return self.servo.wave_left_arm(duration)

    def wave_right_arm(self, duration=2.0):
        """Wave right arm"""
        return self.servo.wave_right_arm(duration)
    
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
    # NEW — Precise-motion delegates
    def drive_distance(self, dist, speed=0.3, engine="internal", timeout=30.0):
        """Drive a specific distance. Delegates to: motion.drive_distance()"""
        return self.motion.drive_distance(dist, speed, engine=engine, timeout=timeout)

    def rotate_angle(self, angle, speed=45.0, engine="internal", timeout=30.0):
        """Rotate by a specific angle. Delegates to: motion.rotate_angle()"""
        return self.motion.rotate_angle(angle, speed, engine=engine, timeout=timeout)

    def drive_and_rotate(self, dist, angle, speed=0.3, turn_speed=45.0, engine="internal", timeout=30.0):
        """Drive then rotate. Delegates to: motion.drive_and_rotate()"""
        return self.motion.drive_and_rotate(dist, angle, speed, turn_speed, engine=engine, timeout=timeout)

    def set_default_engine(self, engine):
        """Set default precise motion engine ('internal' or 'nav2'). Delegates to: motion.set_default_engine()"""
        return self.motion.set_default_engine(engine)

    def is_precise_moving(self):
        """Check if a precise motion action is running. Delegates to: motion.is_precise_moving()"""
        return self.motion.is_precise_moving()

    def set_speed_limits(self, max_linear=None, max_angular=None):
        """Set velocity safety limits (m/s, deg/s). Delegates to: motion.set_speed_limits()"""
        return self.motion.set_speed_limits(max_linear, max_angular)

    def get_speed_limits(self):
        """Get current velocity safety limits. Delegates to: motion.get_speed_limits()"""
        return self.motion.get_speed_limits()

    # NEW — Command queue delegates
    def enqueue_move(self, cmd_list):
        """Push motion commands onto the queue. Delegates to: motion.enqueue_move()"""
        return self.motion.enqueue_move(cmd_list)

    def run_queue(self, block=True):
        """Execute queued commands. Delegates to: motion.run_queue()"""
        return self.motion.run_queue(block)

    def clear_queue(self):
        """Flush queue and stop current motion. Delegates to: motion.clear_queue()"""
        return self.motion.clear_queue()

    def draw_square(self, side_m, speed=0.3, turn_speed=45.0, engine="internal", timeout=30.0):
        """Drive in a square pattern. Delegates to: motion.draw_square()"""
        return self.motion.draw_square(side_m, speed=speed, turn_speed=turn_speed, engine=engine, timeout=timeout)

    # NEW — Vision pipeline delegates
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
