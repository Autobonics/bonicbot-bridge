"""
Servo controller for robot arm, gripper, and neck control
Updated for separate controller groups
"""

import math
import threading
import time

from roslibpy import Topic

from .exceptions import BonicBotError
from .utils import (
    FLOAT64_MULTI_ARRAY_MESSAGE_TYPE,
    JOINT_STATE_MESSAGE_TYPE,
    safe_unadvertise,
    safe_unsubscribe,
)

LEFT_ARM_COMMAND_TOPIC = "/left_arm_controller/commands"
RIGHT_ARM_COMMAND_TOPIC = "/right_arm_controller/commands"
HEAD_COMMAND_TOPIC = "/head_controller/commands"
LEFT_GRIPPER_COMMAND_TOPIC = "/left_gripper_controller/commands"
RIGHT_GRIPPER_COMMAND_TOPIC = "/right_gripper_controller/commands"
JOINT_STATES_TOPIC = "/joint_states"

COMMAND_PUBLISH_DELAY_SECONDS = 0.1
JOINT_STATE_FEEDBACK_DELAY_SECONDS = 0.5
SERVO_ANGLE_ROUND_DIGITS = 2

OPEN_GRIPPER_ANGLE = 60.0
CLOSED_GRIPPER_ANGLE = 0.0
LEFT_NECK_YAW_ANGLE = 90.0
RIGHT_NECK_YAW_ANGLE = -90.0
CENTER_NECK_YAW_ANGLE = 0.0
RESET_SERVO_ANGLE = 0.0

JOINT_NAME_MAP = {
    'left_shoulder_pitch_joint': 'left_shoulder',
    'left_elbow_joint': 'left_elbow',
    'right_shoulder_pitch_joint': 'right_shoulder',
    'right_elbow_joint': 'right_elbow',
    'left_gripper_finger1_joint': 'left_gripper',
    'right_gripper_finger1_joint': 'right_gripper',
    'neck_yaw_joint': 'neck_yaw',
}

LEFT_ARM_LEGACY_JOINTS = ('left_shoulder_pitch_joint', 'left_elbow_joint')
RIGHT_ARM_LEGACY_JOINTS = ('right_shoulder_pitch_joint', 'right_elbow_joint')
GRIPPER_LEGACY_JOINTS = (
    'left_gripper_finger1_joint',
    'right_gripper_finger1_joint',
)
NECK_LEGACY_JOINT = 'neck_yaw_joint'

DEFAULT_SERVO_ANGLES = {
    'left_shoulder': 0.0,
    'left_elbow': 0.0,
    'right_shoulder': 0.0,
    'right_elbow': 0.0,
    'left_gripper': 0.0,
    'right_gripper': 0.0,
    'neck_yaw': 0.0,
}

SIMPLIFIED_JOINT_NAMES = tuple(DEFAULT_SERVO_ANGLES.keys())
VALID_SERVO_INPUT_NAMES = set(JOINT_NAME_MAP) | set(SIMPLIFIED_JOINT_NAMES)

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
        self._angles_lock = threading.Lock()
        
        # Create separate publishers for each controller group
        self.left_arm_pub = Topic(
            self.ros,
            LEFT_ARM_COMMAND_TOPIC,
            FLOAT64_MULTI_ARRAY_MESSAGE_TYPE
        )

        self.right_arm_pub = Topic(
            self.ros,
            RIGHT_ARM_COMMAND_TOPIC,
            FLOAT64_MULTI_ARRAY_MESSAGE_TYPE
        )

        self.head_pub = Topic(
            self.ros,
            HEAD_COMMAND_TOPIC,
            FLOAT64_MULTI_ARRAY_MESSAGE_TYPE
        )

        self.left_gripper_pub = Topic(
            self.ros,
            LEFT_GRIPPER_COMMAND_TOPIC,
            FLOAT64_MULTI_ARRAY_MESSAGE_TYPE
        )

        self.right_gripper_pub = Topic(
            self.ros,
            RIGHT_GRIPPER_COMMAND_TOPIC,
            FLOAT64_MULTI_ARRAY_MESSAGE_TYPE
        )

        # Joint state subscriber for feedback
        self.joint_state_sub = Topic(
            self.ros,
            JOINT_STATES_TOPIC,
            JOINT_STATE_MESSAGE_TYPE
        )

        # Current servo angles (in degrees for user convenience)
        self.current_angles = dict(DEFAULT_SERVO_ANGLES)

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

            # Extract servo angles (convert radians to degrees)
            for i, name in enumerate(names):
                if name in JOINT_NAME_MAP and i < len(positions):
                    simplified_name = JOINT_NAME_MAP[name]
                    radians = positions[i]
                    degrees = math.degrees(radians)
                    with self._angles_lock:
                        self.current_angles[simplified_name] = degrees

        except Exception as exc:
            print(f"⚠️ Error processing joint states: {exc}")

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
            print(f"⚠️ Angle {angle}° for {joint_name} outside limits [{min_angle}°, {max_angle}°], rejecting")
            return None

        return angle

    def _publish_messages(self, *publish_operations):
        for publisher, message in publish_operations:
            publisher.publish(message)

        time.sleep(COMMAND_PUBLISH_DELAY_SECONDS)

    def _get_current_angle(self, joint_name):
        with self._angles_lock:
            return self.current_angles[joint_name]

    def _set_current_angles(self, **angles):
        with self._angles_lock:
            self.current_angles.update(angles)

    def move_left_arm(self, shoulder, elbow, wait=True):
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
            if shoulder is None or elbow is None:
                return False

            # Convert to radians
            shoulder_rad = math.radians(shoulder)
            elbow_rad = math.radians(elbow)
            
            # Publish command [shoulder, elbow]
            msg = {'data': [shoulder_rad, elbow_rad]}
            self._publish_messages((self.left_arm_pub, msg))

            # Update internal state
            self._set_current_angles(left_shoulder=shoulder, left_elbow=elbow)

            return True

        except Exception as exc:
            raise BonicBotError(f"Failed to move left arm: {str(exc)}")

    def move_right_arm(self, shoulder, elbow, wait=True):
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
            if shoulder is None or elbow is None:
                return False

            # Convert to radians
            shoulder_rad = math.radians(shoulder)
            elbow_rad = math.radians(elbow)
            
            # Publish command [shoulder, elbow]
            msg = {'data': [shoulder_rad, elbow_rad]}
            self._publish_messages((self.right_arm_pub, msg))

            # Update internal state
            self._set_current_angles(right_shoulder=shoulder, right_elbow=elbow)

            return True

        except Exception as exc:
            raise BonicBotError(f"Failed to move right arm: {str(exc)}")

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
            if left is None or right is None:
                return False

            # Convert to radians
            left_rad = math.radians(left)
            right_rad = math.radians(right)
            
            # Publish to both grippers
            left_msg = {'data': [left_rad]}
            right_msg = {'data': [right_rad]}
            self._publish_messages(
                (self.left_gripper_pub, left_msg),
                (self.right_gripper_pub, right_msg),
            )

            # Update internal state
            self._set_current_angles(left_gripper=left, right_gripper=right)

            return True

        except Exception as exc:
            raise BonicBotError(f"Failed to set grippers: {str(exc)}")

    def open_grippers(self):
        """
        Open both grippers fully

        Returns:
            bool: True if command sent successfully
        """
        return self.set_grippers(OPEN_GRIPPER_ANGLE, OPEN_GRIPPER_ANGLE)

    def close_grippers(self):
        """
        Close both grippers

        Returns:
            bool: True if command sent successfully
        """
        return self.set_grippers(CLOSED_GRIPPER_ANGLE, CLOSED_GRIPPER_ANGLE)

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
            if angle is None:
                return False

            # Convert to radians
            angle_rad = math.radians(angle)
            
            # Publish to left gripper
            msg = {'data': [angle_rad]}
            self._publish_messages((self.left_gripper_pub, msg))

            # Update internal state
            self._set_current_angles(left_gripper=angle)

            return True

        except Exception as exc:
            raise BonicBotError(f"Failed to set left gripper: {str(exc)}")

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
            if angle is None:
                return False

            # Convert to radians
            angle_rad = math.radians(angle)
            
            # Publish to right gripper
            msg = {'data': [angle_rad]}
            self._publish_messages((self.right_gripper_pub, msg))

            # Update internal state
            self._set_current_angles(right_gripper=angle)

            return True

        except Exception as exc:
            raise BonicBotError(f"Failed to set right gripper: {str(exc)}")

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
            if yaw is None:
                return False

            # Convert to radians
            yaw_rad = math.radians(yaw)
            
            # Publish command [yaw]
            msg = {'data': [yaw_rad]}
            self._publish_messages((self.head_pub, msg))

            # Update internal state
            self._set_current_angles(neck_yaw=yaw)

            return True

        except Exception as exc:
            raise BonicBotError(f"Failed to set neck: {str(exc)}")

    def look_left(self):
        """
        Turn neck fully left

        Returns:
            bool: True if command sent successfully
        """
        return self.set_neck(LEFT_NECK_YAW_ANGLE)

    def look_right(self):
        """
        Turn neck fully right

        Returns:
            bool: True if command sent successfully
        """
        return self.set_neck(RIGHT_NECK_YAW_ANGLE)

    def look_center(self):
        """
        Center the neck

        Returns:
            bool: True if command sent successfully
        """
        return self.set_neck(CENTER_NECK_YAW_ANGLE)

    def reset_all_servos(self):
        """
        Reset all servos to neutral position (0 degrees)

        Returns:
            bool: True if command sent successfully
        """
        reset_rad = math.radians(RESET_SERVO_ANGLE)
        self._publish_messages(
            (self.left_arm_pub, {'data': [reset_rad, reset_rad]}),
            (self.right_arm_pub, {'data': [reset_rad, reset_rad]}),
            (self.left_gripper_pub, {'data': [reset_rad]}),
            (self.right_gripper_pub, {'data': [reset_rad]}),
            (self.head_pub, {'data': [reset_rad]}),
        )
        self._set_current_angles(**DEFAULT_SERVO_ANGLES)
        return True

    def get_servo_angles(self):
        """
        Get current servo angles

        Note: Includes small delay to ensure joint state feedback has updated

        Returns:
            dict: Current angles in degrees for all servos (rounded to 2 decimal places)
        """
        # Wait for joint state feedback to update
        time.sleep(JOINT_STATE_FEEDBACK_DELAY_SECONDS)

        # Round all values to 2 decimal places for cleaner output
        with self._angles_lock:
            return {
                joint: round(angle, SERVO_ANGLE_ROUND_DIGITS)
                for joint, angle in self.current_angles.items()
            }

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
        invalid_joints = set(angles) - VALID_SERVO_INPUT_NAMES
        if invalid_joints:
            return False

        # Map old joint names to new methods
        if any(joint_name in angles for joint_name in LEFT_ARM_LEGACY_JOINTS + ('left_shoulder', 'left_elbow')):
            shoulder = angles.get(
                LEFT_ARM_LEGACY_JOINTS[0],
                angles.get('left_shoulder', self._get_current_angle('left_shoulder')),
            )
            elbow = angles.get(
                LEFT_ARM_LEGACY_JOINTS[1],
                angles.get('left_elbow', self._get_current_angle('left_elbow')),
            )
            result = result and self.move_left_arm(shoulder, elbow)

        if any(joint_name in angles for joint_name in RIGHT_ARM_LEGACY_JOINTS + ('right_shoulder', 'right_elbow')):
            shoulder = angles.get(
                RIGHT_ARM_LEGACY_JOINTS[0],
                angles.get('right_shoulder', self._get_current_angle('right_shoulder')),
            )
            elbow = angles.get(
                RIGHT_ARM_LEGACY_JOINTS[1],
                angles.get('right_elbow', self._get_current_angle('right_elbow')),
            )
            result = result and self.move_right_arm(shoulder, elbow)

        if any(joint_name in angles for joint_name in GRIPPER_LEGACY_JOINTS + ('left_gripper', 'right_gripper')):
            left = angles.get(
                GRIPPER_LEGACY_JOINTS[0],
                angles.get('left_gripper', self._get_current_angle('left_gripper')),
            )
            right = angles.get(
                GRIPPER_LEGACY_JOINTS[1],
                angles.get('right_gripper', self._get_current_angle('right_gripper')),
            )
            result = result and self.set_grippers(left, right)

        if NECK_LEGACY_JOINT in angles or 'neck_yaw' in angles:
            result = result and self.set_neck(
                angles.get(NECK_LEGACY_JOINT, angles.get('neck_yaw'))
            )

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
        simplified_name = JOINT_NAME_MAP.get(joint_name, joint_name)

        if simplified_name not in self.current_angles:
            raise BonicBotError(f"Unknown servo joint: {joint_name}")

        return self._get_current_angle(simplified_name)

    def shutdown(self):
        """Release servo topics during teardown."""
        safe_unsubscribe(self.joint_state_sub)

        for publisher in (
            self.left_arm_pub,
            self.right_arm_pub,
            self.head_pub,
            self.left_gripper_pub,
            self.right_gripper_pub,
        ):
            safe_unadvertise(publisher)
