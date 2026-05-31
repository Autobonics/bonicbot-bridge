"""
Shared utilities for BonicBot controllers
"""

from roslibpy import ServiceRequest

# Common message types
TRIGGER_SERVICE_TYPE = "std_srvs/Trigger"
STRING_MESSAGE_TYPE = "std_msgs/String"
BOOL_MESSAGE_TYPE = "std_msgs/Bool"
FLOAT32_MESSAGE_TYPE = "std_msgs/Float32"
FLOAT64_MULTI_ARRAY_MESSAGE_TYPE = "std_msgs/Float64MultiArray"
TWIST_MESSAGE_TYPE = "geometry_msgs/Twist"
POSE_STAMPED_MESSAGE_TYPE = "geometry_msgs/PoseStamped"
POSE_WITH_COVARIANCE_MESSAGE_TYPE = "geometry_msgs/PoseWithCovarianceStamped"
ODOMETRY_MESSAGE_TYPE = "nav_msgs/Odometry"
CAMERA_INFO_MESSAGE_TYPE = "sensor_msgs/CameraInfo"
COMPRESSED_IMAGE_MESSAGE_TYPE = "sensor_msgs/CompressedImage"
JOINT_STATE_MESSAGE_TYPE = "sensor_msgs/JointState"
OCCUPANCY_GRID_MESSAGE_TYPE = "nav_msgs/OccupancyGrid"

# Mapping-related topics
MAP_TOPIC = "/map"
MAP_AVAILABLE_TOPIC = "/robot/map_available"

# Shared navigation service topics (used by both motion.py and system.py)
START_NAVIGATION_SERVICE = "/robot/start_navigation"
STOP_NAVIGATION_SERVICE = "/robot/stop_navigation"

# Shared service-response sentinel strings (used by both motion.py and system.py)
INACTIVE_RESPONSE_TEXT = "not active"
ALREADY_ACTIVE_RESPONSE_TEXT = "already active"

# Shared timeout for long-running navigation service calls (used by both motion.py and system.py)
NAVIGATION_SERVICE_CALL_TIMEOUT_SECONDS = 65.0

# Maximum distance (metres) allowed for a single drive_distance() call.
# Prevents accidental long-distance drives that risk collisions.
MAX_PRECISE_DISTANCE = 2.0

def call_trigger_service(service, timeout_secs, exception_cls, error_prefix):
    """
    Utility to call a standard trigger service.
    
    Args:
        service: Initialized roslibpy.Service
        timeout_secs: Service call timeout in seconds
        exception_cls: Exception class to raise on failure
        error_prefix: Prefix for the error message
        
    Returns:
        The response dictionary from the service call.
    """
    try:
        request = ServiceRequest()
        response = service.call(request, timeout=timeout_secs)
        
        if not response['success']:
            # Gracefully handle the case where the service is already active
            if ALREADY_ACTIVE_RESPONSE_TEXT in response['message'].lower():
                print(f"ℹ️ {error_prefix} (skipped): {response['message']}")
            else:
                raise exception_cls(f"{error_prefix}: {response['message']}")
            
        return response
    except Exception as exc:
        if isinstance(exc, exception_cls):
            raise
        raise exception_cls(f"{error_prefix} failed: {str(exc)}")

def safe_unsubscribe(topic):
    """Safely unsubscribe from a topic, catching and logging any errors."""
    if topic:
        try:
            topic.unsubscribe()
        except Exception as exc:
            print(f"⚠️ Error unsubscribing from topic {getattr(topic, 'name', 'unknown')}: {exc}")

def safe_unadvertise(publisher):
    """Safely unadvertise a publisher, catching and logging any errors."""
    if publisher:
        try:
            publisher.unadvertise()
        except Exception as exc:
            print(f"⚠️ Error unadvertising publisher {getattr(publisher, 'name', 'unknown')}: {exc}")

# ── Odometry topics ───────────────────────────────────────────────────────
# DIFF_CONT_ODOM_TOPIC is retained as a named constant so its
# former users are explicit about the frame they are targeting.
# New code should prefer ODOMETRY_FILTERED_TOPIC (EKF, less drift).
DIFF_CONT_ODOM_TOPIC           = "/diff_cont/odom"
ODOMETRY_FILTERED_TOPIC        = "/odometry/filtered"

# ── Robot state & status topics ───────────────────────────────────────────
ROBOT_STATE_TOPIC              = "/robot/state"
MAPPING_STATUS_TOPIC           = "/robot/mapping_active"
NAVIGATION_STATUS_TOPIC        = "/robot/navigation_active"
CAMERA_STATUS_TOPIC            = "/robot/camera_active"
CURRENT_GOAL_TOPIC             = "/robot/current_goal"
DISTANCE_TO_GOAL_TOPIC         = "/robot/distance_to_goal"
LOCATIONS_LIST_TOPIC           = "/robot/locations_list"
NAV_STATUS_TOPIC               = "/robot/nav_status"
EXPLORE_STATUS_TOPIC           = "/robot/explore_active"

# ── Robot action publisher topics ─────────────────────────────────────────
GOTO_LOCATION_TOPIC            = "/robot/goto_location"
SAVE_LOCATION_TOPIC            = "/robot/save_location"
DELETE_LOCATION_TOPIC          = "/robot/delete_location"

# ── Navigation topics ────────────────────────────────────────────────────
CMD_VEL_TOPIC                  = "/cmd_vel"
GOAL_POSE_TOPIC                = "/goal_pose"
INITIAL_POSE_TOPIC             = "/initialpose"

# ── Mapping & costmap topics ─────────────────────────────────────────────
NAV2_COSTMAP_TOPIC             = "/global_costmap/costmap"
EXPLORE_FRONTIERS_TOPIC        = "/explore/frontiers"

# ── Mapping services ─────────────────────────────────────────────────────
START_MAPPING_SERVICE          = "/robot/start_mapping"
STOP_MAPPING_SERVICE           = "/robot/stop_mapping"
SAVE_MAP_SERVICE               = "/robot/save_map"

# ── Camera services ──────────────────────────────────────────────────────
START_CAMERA_SERVICE           = "/robot/start_camera"
STOP_CAMERA_SERVICE            = "/robot/stop_camera"

# ── Exploration service ──────────────────────────────────────────────────
START_EXPLORE_SERVICE          = "/robot/start_explore"

# ── Navigation action service ────────────────────────────────────────────
CANCEL_NAVIGATION_SERVICE      = "/robot/cancel_navigation"

# ── Camera sensor topics ─────────────────────────────────────────────────
CAMERA_INFO_TOPIC              = "/camera/camera_info"
COMPRESSED_IMAGE_TOPIC         = "/camera/image_raw/compressed"

# ── Servo / joint controller topics ──────────────────────────────────────
LEFT_ARM_COMMAND_TOPIC         = "/left_arm_controller/commands"
RIGHT_ARM_COMMAND_TOPIC        = "/right_arm_controller/commands"
HEAD_COMMAND_TOPIC             = "/head_controller/commands"
LEFT_GRIPPER_COMMAND_TOPIC     = "/left_gripper_controller/commands"
RIGHT_GRIPPER_COMMAND_TOPIC    = "/right_gripper_controller/commands"
JOINT_STATES_TOPIC             = "/joint_states"

# ── Vision pipeline topics ───────────────────────────────────────────────
VISION_CONTROL_TOPIC           = "/vision/control"
VISION_MODE_TOPIC              = "/robot/vision_mode"
VISION_DETECTIONS_TOPIC        = "/vision/detections"
VISION_FACES_TOPIC             = "/vision/faces"
VISION_POSE_TOPIC              = "/vision/pose"
VISION_GESTURE_TOPIC           = "/vision/gesture"
VISION_ARUCO_TOPIC             = "/vision/aruco"

# ── Additional message types ─────────────────────────────────────────────
MARKER_ARRAY_MESSAGE_TYPE      = "visualization_msgs/MarkerArray"
GOAL_STATUS_ARRAY_MESSAGE_TYPE = "action_msgs/GoalStatusArray"
# Note: VISION_MESSAGE_TYPE = STRING_MESSAGE_TYPE — do NOT add a duplicate;
# vision.py must import STRING_MESSAGE_TYPE and use it directly.
