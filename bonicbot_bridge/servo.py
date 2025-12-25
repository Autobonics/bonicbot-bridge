"""
Servo controller for robot arm, gripper, and neck control
"""

import math
from roslibpy import Topic
from .exceptions import BonicBotError

# Servo joint names and their limits (min, max) in degrees
SERVO_JOINTS = {
    'left_shoulder_pitch_joint': (-45.0, 180.0),
    'left_elbow_joint': (0.0, 50.0),
    'right_shoulder_pitch_joint': (-45.0, 180.0),
    'right_elbow_joint': (0.0, 50.0),
    'left_gripper_finger1_joint': (-45.0, 60.0),
    'right_gripper_finger1_joint': (-45.0, 60.0),
    'neck_yaw_joint': (-90.0, 90.0),
}

# Order of servos in the command array
SERVO_ORDER = [
    'left_shoulder_pitch_joint',
    'left_elbow_joint',
    'right_shoulder_pitch_joint',
    'right_elbow_joint',
    'left_gripper_finger1_joint',
    'right_gripper_finger1_joint',
    'neck_yaw_joint',
]

class ServoController:
    def __init__(self, ros_client):
        """
        Initialize servo controller
        
        Args:
            ros_client: Connected roslibpy Ros instance
        """
        self.ros = ros_client
        
        # Servo command publisher
        self.servo_pub = Topic(
            self.ros,
            '/servo_position_controller/commands',
            'std_msgs/Float64MultiArray'
        )
        
        # Joint state subscriber
        self.joint_state_sub = Topic(
            self.ros,
            '/joint_states',
            'sensor_msgs/JointState'
        )
        
        # Current servo angles (in degrees)
        self.current_angles = {joint: 0.0 for joint in SERVO_JOINTS}
        
        # Subscribe to joint states for feedback
        self.joint_state_sub.subscribe(self._joint_state_callback)
        
        # Advertise servo publisher
        self.servo_pub.advertise()
    
    def _joint_state_callback(self, msg):
        """
        Update current servo positions from joint states
        
        Args:
            msg: JointState message from ROS
        """
        try:
            names = msg.get('name', [])
            positions = msg.get('position', [])
            
            # Extract servo angles (convert radians to degrees)
            for i, name in enumerate(names):
                if name in self.current_angles and i < len(positions):
                    radians = positions[i]
                    degrees = math.degrees(radians)
                    self.current_angles[name] = degrees
                    
        except Exception as e:
            print(f"⚠️ Error processing joint states: {e}")
    
    def _validate_angle(self, joint_name, angle):
        """
        Validate and clamp servo angle to hardware limits
        
        Args:
            joint_name: Name of the servo joint
            angle: Target angle in degrees
            
        Returns:
            float: Clamped angle within valid range
        """
        if joint_name not in SERVO_JOINTS:
            raise BonicBotError(f"Unknown servo joint: {joint_name}")
        
        min_angle, max_angle = SERVO_JOINTS[joint_name]
        
        if angle < min_angle or angle > max_angle:
            print(f"⚠️ Angle {angle}° for {joint_name} outside limits [{min_angle}°, {max_angle}°], clamping")
            angle = max(min_angle, min(max_angle, angle))
        
        return angle
    
    def set_servo_angles(self, angles):
        """
        Set multiple servo angles
        
        Args:
            angles: Dictionary mapping joint names to target angles in degrees
                   Example: {'left_shoulder_pitch_joint': 45.0, 'neck_yaw_joint': -30.0}
                   
        Returns:
            bool: True if command sent successfully
        """
        try:
            # Start with current angles
            target_angles = dict(self.current_angles)
            
            # Update with requested angles (validate each)
            for joint_name, angle in angles.items():
                validated_angle = self._validate_angle(joint_name, angle)
                target_angles[joint_name] = validated_angle
            
            # Build command array in correct order (convert degrees to radians)
            command_data = [
                math.radians(target_angles[joint])
                for joint in SERVO_ORDER
            ]
            
            # Publish command
            msg = {'data': command_data}
            self.servo_pub.publish(msg)
            
            # Update internal state
            self.current_angles.update(target_angles)
            
            return True
            
        except Exception as e:
            raise BonicBotError(f"Failed to set servo angles: {str(e)}")
    
    def set_single_servo(self, joint_name, angle):
        """
        Set a single servo angle
        
        Args:
            joint_name: Name of the servo joint
            angle: Target angle in degrees
            
        Returns:
            bool: True if command sent successfully
        """
        return self.set_servo_angles({joint_name: angle})
    
    def get_servo_angles(self):
        """
        Get current servo angles
        
        Returns:
            dict: Current angles in degrees for all servos
        """
        return dict(self.current_angles)
    
    def get_single_servo(self, joint_name):
        """
        Get a single servo's current angle
        
        Args:
            joint_name: Name of the servo joint
            
        Returns:
            float: Current angle in degrees
        """
        if joint_name not in self.current_angles:
            raise BonicBotError(f"Unknown servo joint: {joint_name}")
        
        return self.current_angles[joint_name]
    
    def move_left_arm(self, shoulder, elbow):
        """
        Move left arm (shoulder and elbow)
        
        Args:
            shoulder: Shoulder pitch angle in degrees (-45 to 180)
            elbow: Elbow angle in degrees (0 to 50)
            
        Returns:
            bool: True if command sent successfully
        """
        return self.set_servo_angles({
            'left_shoulder_pitch_joint': shoulder,
            'left_elbow_joint': elbow,
        })
    
    def move_right_arm(self, shoulder, elbow):
        """
        Move right arm (shoulder and elbow)
        
        Args:
            shoulder: Shoulder pitch angle in degrees (-45 to 180)
            elbow: Elbow angle in degrees (0 to 50)
            
        Returns:
            bool: True if command sent successfully
        """
        return self.set_servo_angles({
            'right_shoulder_pitch_joint': shoulder,
            'right_elbow_joint': elbow,
        })
    
    def set_grippers(self, left, right):
        """
        Control both gripper fingers
        
        Args:
            left: Left gripper angle in degrees (-45 to 60)
            right: Right gripper angle in degrees (-45 to 60)
            
        Returns:
            bool: True if command sent successfully
        """
        return self.set_servo_angles({
            'left_gripper_finger1_joint': left,
            'right_gripper_finger1_joint': right,
        })
    
    def open_grippers(self):
        """
        Open both grippers fully
        
        Returns:
            bool: True if command sent successfully
        """
        return self.set_grippers(60.0, 60.0)
    
    def close_grippers(self):
        """
        Close both grippers
        
        Returns:
            bool: True if command sent successfully
        """
        return self.set_grippers(-45.0, -45.0)
    
    def set_neck(self, yaw):
        """
        Set neck yaw angle
        
        Args:
            yaw: Neck yaw angle in degrees (-90 to 90)
            
        Returns:
            bool: True if command sent successfully
        """
        return self.set_single_servo('neck_yaw_joint', yaw)
    
    def look_left(self):
        """
        Turn neck fully left
        
        Returns:
            bool: True if command sent successfully
        """
        return self.set_neck(90.0)
    
    def look_right(self):
        """
        Turn neck fully right
        
        Returns:
            bool: True if command sent successfully
        """
        return self.set_neck(-90.0)
    
    def look_center(self):
        """
        Center the neck
        
        Returns:
            bool: True if command sent successfully
        """
        return self.set_neck(0.0)
    
    def reset_all_servos(self):
        """
        Reset all servos to neutral position (0 degrees)
        
        Returns:
            bool: True if command sent successfully
        """
        neutral_angles = {joint: 0.0 for joint in SERVO_JOINTS}
        return self.set_servo_angles(neutral_angles)
    
    def wave_left_arm(self, duration=2.0):
        """
        Wave the left arm (demo motion)
        
        Args:
            duration: Duration of wave in seconds
            
        Returns:
            bool: True if motion completed
        """
        import time
        
        # Wave motion
        self.move_left_arm(90, 30)
        time.sleep(duration / 4)
        self.move_left_arm(45, 10)
        time.sleep(duration / 4)
        self.move_left_arm(90, 30)
        time.sleep(duration / 4)
        self.move_left_arm(0, 0)
        time.sleep(duration / 4)
        
        return True
    
    def wave_right_arm(self, duration=2.0):
        """
        Wave the right arm (demo motion)
        
        Args:
            duration: Duration of wave in seconds
            
        Returns:
            bool: True if motion completed
        """
        import time
        
        # Wave motion
        self.move_right_arm(90, 30)
        time.sleep(duration / 4)
        self.move_right_arm(45, 10)
        time.sleep(duration / 4)
        self.move_right_arm(90, 30)
        time.sleep(duration / 4)
        self.move_right_arm(0, 0)
        time.sleep(duration / 4)
        
        return True
    
    def get_servo_limits(self, joint_name=None):
        """
        Get servo angle limits
        
        Args:
            joint_name: Optional joint name. If None, returns all limits.
            
        Returns:
            dict or tuple: If joint_name is None, returns dict of all limits.
                          Otherwise returns (min, max) tuple for specified joint.
        """
        if joint_name is None:
            return dict(SERVO_JOINTS)
        
        if joint_name not in SERVO_JOINTS:
            raise BonicBotError(f"Unknown servo joint: {joint_name}")
        
        return SERVO_JOINTS[joint_name]
