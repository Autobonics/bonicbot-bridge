"""
Servo controller for robot arm, gripper, and neck control
Updated for separate controller groups
"""

import math
from roslibpy import Topic
from .exceptions import BonicBotError

# Servo joint limits (min, max) in degrees
# Note: User API uses degrees, but ROS topics use radians
SERVO_LIMITS = {
    # Arms (shoulder, elbow)
    'left_shoulder': (-45.0, 180.0),
    'left_elbow': (0.0, 50.0),
    'right_shoulder': (-45.0, 180.0),
    'right_elbow': (0.0, 50.0),
    # Grippers
    'left_gripper': (-45.0, 60.0),
    'right_gripper': (-45.0, 60.0),
    # Head
    'neck_yaw': (-90.0, 90.0),
}

class ServoController:
    def __init__(self, ros_client):
        """
        Initialize servo controller with separate group publishers
        
        Args:
            ros_client: Connected roslibpy Ros instance
        """
        self.ros = ros_client
        
        # Create separate publishers for each controller group
        self.left_arm_pub = Topic(
            self.ros,
            '/left_arm_controller/commands',
            'std_msgs/Float64MultiArray'
        )
        
        self.right_arm_pub = Topic(
            self.ros,
            '/right_arm_controller/commands',
            'std_msgs/Float64MultiArray'
        )
        
        self.head_pub = Topic(
            self.ros,
            '/head_controller/commands',
            'std_msgs/Float64MultiArray'
        )
        
        self.left_gripper_pub = Topic(
            self.ros,
            '/left_gripper_controller/commands',
            'std_msgs/Float64MultiArray'
        )
        
        self.right_gripper_pub = Topic(
            self.ros,
            '/right_gripper_controller/commands',
            'std_msgs/Float64MultiArray'
        )
        
        # Joint state subscriber for feedback
        self.joint_state_sub = Topic(
            self.ros,
            '/joint_states',
            'sensor_msgs/JointState'
        )
        
        # Current servo angles (in degrees for user convenience)
        self.current_angles = {
            'left_shoulder': 0.0,
            'left_elbow': 0.0,
            'right_shoulder': 0.0,
            'right_elbow': 0.0,
            'left_gripper': 0.0,
            'right_gripper': 0.0,
            'neck_yaw': 0.0,
        }
        
        # Subscribe to joint states for feedback
        self.joint_state_sub.subscribe(self._joint_state_callback)
        
        # Advertise all publishers
        self.left_arm_pub.advertise()
        self.right_arm_pub.advertise()
        self.head_pub.advertise()
        self.left_gripper_pub.advertise()
        self.right_gripper_pub.advertise()
    
    def _joint_state_callback(self, msg):
        """
        Update current servo positions from joint states
        
        Args:
            msg: JointState message from ROS
        """
        try:
            names = msg.get('name', [])
            positions = msg.get('position', [])
            
            # Map ROS joint names to our simplified names
            joint_map = {
                'left_shoulder_pitch_joint': 'left_shoulder',
                'left_elbow_joint': 'left_elbow',
                'right_shoulder_pitch_joint': 'right_shoulder',
                'right_elbow_joint': 'right_elbow',
                'left_gripper_finger1_joint': 'left_gripper',
                'right_gripper_finger1_joint': 'right_gripper',
                'neck_yaw_joint': 'neck_yaw',
            }
            
            # Extract servo angles (convert radians to degrees)
            for i, name in enumerate(names):
                if name in joint_map and i < len(positions):
                    simplified_name = joint_map[name]
                    radians = positions[i]
                    degrees = math.degrees(radians)
                    self.current_angles[simplified_name] = degrees
                    
        except Exception as e:
            print(f"⚠️ Error processing joint states: {e}")
    
    def _validate_angle(self, joint_name, angle):
        """
        Validate and clamp servo angle to hardware limits
        
        Args:
            joint_name: Simplified joint name (e.g. 'left_shoulder')
            angle: Target angle in degrees
            
        Returns:
            float: Clamped angle within valid range
        """
        if joint_name not in SERVO_LIMITS:
            raise BonicBotError(f"Unknown servo joint: {joint_name}")
        
        min_angle, max_angle = SERVO_LIMITS[joint_name]
        
        if angle < min_angle or angle > max_angle:
            print(f"⚠️ Angle {angle}° for {joint_name} outside limits [{min_angle}°, {max_angle}°], clamping")
            angle = max(min_angle, min(max_angle, angle))
        
        return angle
    
    def move_left_arm(self, shoulder, elbow):
        """
        Move left arm (shoulder and elbow)
        
        Args:
            shoulder: Shoulder pitch angle in degrees (-45 to 180)
            elbow: Elbow angle in degrees (0 to 50)
            
        Returns:
            bool: True if command sent successfully
        """
        try:
            # Validate angles
            shoulder = self._validate_angle('left_shoulder', shoulder)
            elbow = self._validate_angle('left_elbow', elbow)
            
            # Convert to radians
            shoulder_rad = math.radians(shoulder)
            elbow_rad = math.radians(elbow)
            
            # Publish command [shoulder, elbow]
            msg = {'data': [shoulder_rad, elbow_rad]}
            self.left_arm_pub.publish(msg)
            
            # Update internal state
            self.current_angles['left_shoulder'] = shoulder
            self.current_angles['left_elbow'] = elbow
            
            return True
            
        except Exception as e:
            raise BonicBotError(f"Failed to move left arm: {str(e)}")
    
    def move_right_arm(self, shoulder, elbow):
        """
        Move right arm (shoulder and elbow)
        
        Args:
            shoulder: Shoulder pitch angle in degrees (-45 to 180)
            elbow: Elbow angle in degrees (0 to 50)
            
        Returns:
            bool: True if command sent successfully
        """
        try:
            # Validate angles
            shoulder = self._validate_angle('right_shoulder', shoulder)
            elbow = self._validate_angle('right_elbow', elbow)
            
            # Convert to radians
            shoulder_rad = math.radians(shoulder)
            elbow_rad = math.radians(elbow)
            
            # Publish command [shoulder, elbow]
            msg = {'data': [shoulder_rad, elbow_rad]}
            self.right_arm_pub.publish(msg)
            
            # Update internal state
            self.current_angles['right_shoulder'] = shoulder
            self.current_angles['right_elbow'] = elbow
            
            return True
            
        except Exception as e:
            raise BonicBotError(f"Failed to move right arm: {str(e)}")
    
    def set_grippers(self, left, right):
        """
        Control both gripper fingers
        
        Args:
            left: Left gripper angle in degrees (-28.6 to 60)
            right: Right gripper angle in degrees (-28.6 to 60)
            
        Returns:
            bool: True if command sent successfully
        """
        try:
            # Validate angles
            left = self._validate_angle('left_gripper', left)
            right = self._validate_angle('right_gripper', right)
            
            # Convert to radians
            left_rad = math.radians(left)
            right_rad = math.radians(right)
            
            # Publish to both grippers
            left_msg = {'data': [left_rad]}
            right_msg = {'data': [right_rad]}
            
            self.left_gripper_pub.publish(left_msg)
            self.right_gripper_pub.publish(right_msg)
            
            # Update internal state
            self.current_angles['left_gripper'] = left
            self.current_angles['right_gripper'] = right
            
            return True
            
        except Exception as e:
            raise BonicBotError(f"Failed to set grippers: {str(e)}")
    
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
        return self.set_grippers(0.0, 0.0)
    
    def set_left_gripper(self, angle):
        """
        Control left gripper only
        
        Args:
            angle: Left gripper angle in degrees (-45 to 60)
            
        Returns:
            bool: True if command sent successfully
        """
        try:
            # Validate angle
            angle = self._validate_angle('left_gripper', angle)
            
            # Convert to radians
            angle_rad = math.radians(angle)
            
            # Publish to left gripper
            msg = {'data': [angle_rad]}
            self.left_gripper_pub.publish(msg)
            
            # Update internal state
            self.current_angles['left_gripper'] = angle
            
            return True
            
        except Exception as e:
            raise BonicBotError(f"Failed to set left gripper: {str(e)}")
    
    def set_right_gripper(self, angle):
        """
        Control right gripper only
        
        Args:
            angle: Right gripper angle in degrees (-45 to 60)
            
        Returns:
            bool: True if command sent successfully
        """
        try:
            # Validate angle
            angle = self._validate_angle('right_gripper', angle)
            
            # Convert to radians
            angle_rad = math.radians(angle)
            
            # Publish to right gripper
            msg = {'data': [angle_rad]}
            self.right_gripper_pub.publish(msg)
            
            # Update internal state
            self.current_angles['right_gripper'] = angle
            
            return True
            
        except Exception as e:
            raise BonicBotError(f"Failed to set right gripper: {str(e)}")
    
    def set_neck(self, yaw):
        """
        Set neck yaw angle
        
        Args:
            yaw: Neck yaw angle in degrees (-90 to 90)
            
        Returns:
            bool: True if command sent successfully
        """
        try:
            # Validate angle
            yaw = self._validate_angle('neck_yaw', yaw)
            
            # Convert to radians
            yaw_rad = math.radians(yaw)
            
            # Publish command [yaw]
            msg = {'data': [yaw_rad]}
            self.head_pub.publish(msg)
            
            # Update internal state
            self.current_angles['neck_yaw'] = yaw
            
            return True
            
        except Exception as e:
            raise BonicBotError(f"Failed to set neck: {str(e)}")
    
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
        self.move_left_arm(0.0, 0.0)
        self.move_right_arm(0.0, 0.0)
        self.set_grippers(0.0, 0.0)
        self.set_neck(0.0)
        return True
    
    def get_servo_angles(self):
        """
        Get current servo angles
        
        Returns:
            dict: Current angles in degrees for all servos
        """
        return dict(self.current_angles)
    
    def get_servo_limits(self):
        """
        Get servo angle limits
        
        Returns:
            dict: Dictionary of (min, max) tuples for each joint
        """
        return dict(SERVO_LIMITS)
    
    # Legacy compatibility methods (deprecated - kept for backward compatibility)
    def set_servo_angles(self, angles):
        """
        Legacy method - now maps to individual controller calls
        
        Deprecated: Use move_left_arm(), move_right_arm(), set_grippers(), set_neck() instead
        """
        result = True
        
        # Map old joint names to new methods
        if 'left_shoulder_pitch_joint' in angles or 'left_elbow_joint' in angles:
            shoulder = angles.get('left_shoulder_pitch_joint', self.current_angles['left_shoulder'])
            elbow = angles.get('left_elbow_joint', self.current_angles['left_elbow'])
            result = result and self.move_left_arm(shoulder, elbow)
        
        if 'right_shoulder_pitch_joint' in angles or 'right_elbow_joint' in angles:
            shoulder = angles.get('right_shoulder_pitch_joint', self.current_angles['right_shoulder'])
            elbow = angles.get('right_elbow_joint', self.current_angles['right_elbow'])
            result = result and self.move_right_arm(shoulder, elbow)
        
        if 'left_gripper_finger1_joint' in angles or 'right_gripper_finger1_joint' in angles:
            left = angles.get('left_gripper_finger1_joint', self.current_angles['left_gripper'])
            right = angles.get('right_gripper_finger1_joint', self.current_angles['right_gripper'])
            result = result and self.set_grippers(left, right)
        
        if 'neck_yaw_joint' in angles:
            result = result and self.set_neck(angles['neck_yaw_joint'])
        
        return result
    
    def set_single_servo(self, joint_name, angle):
        """
        Legacy method - maps old joint names to new controller calls
        
        Deprecated: Use move_left_arm(), move_right_arm(), set_grippers(), set_neck() instead
        """
        return self.set_servo_angles({joint_name: angle})
    
    def get_single_servo(self, joint_name):
        """
        Get a single servo's current angle (legacy compatibility)
        
        Args:
            joint_name: Old-style joint name or new simplified name
        """
        # Map old names to new names
        name_map = {
            'left_shoulder_pitch_joint': 'left_shoulder',
            'left_elbow_joint': 'left_elbow',
            'right_shoulder_pitch_joint': 'right_shoulder',
            'right_elbow_joint': 'right_elbow',
            'left_gripper_finger1_joint': 'left_gripper',
            'right_gripper_finger1_joint': 'right_gripper',
            'neck_yaw_joint': 'neck_yaw',
        }
        
        simplified_name = name_map.get(joint_name, joint_name)
        
        if simplified_name not in self.current_angles:
            raise BonicBotError(f"Unknown servo joint: {joint_name}")
        
        return self.current_angles[simplified_name]
