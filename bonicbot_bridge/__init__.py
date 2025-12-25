"""
BonicBot Bridge - Python SDK for educational robotics programming
Provides high-level API for controlling BonicBot via ROS2 rosbridge
"""

from .core import BonicBot
from .camera import CameraManager
from .servo import ServoController
from .exceptions import BonicBotError, ConnectionError, NavigationError

__version__ = "0.2.0"
__author__ = "Autobonics Pvt Ltd"

__all__ = [
    "BonicBot", 
    "CameraManager", 
    "ServoController",
    "BonicBotError", 
    "ConnectionError", 
    "NavigationError"
]