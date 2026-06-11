"""
BonicBot Bridge - Python SDK for educational robotics programming
Provides high-level API for controlling BonicBot via ROS2 rosbridge
"""

from .camera import CameraManager
from .core import BonicBot
from .exceptions import BonicBotError, ConnectionError, NavigationError, PreciseMotionError
from .precisemotion import PreciseMotionEngine
from .servo import ServoController
from .vision import VisionController

__version__ = "0.2.0"
__author__ = "Autobonics Pvt Ltd"

__all__ = [
    "BonicBot",
    "CameraManager",
    "ServoController",
    "VisionController",
    "BonicBotError",
    "ConnectionError",
    "NavigationError",
    "PreciseMotionError",
    "PreciseMotionEngine",
]
