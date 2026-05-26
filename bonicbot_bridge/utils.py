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
