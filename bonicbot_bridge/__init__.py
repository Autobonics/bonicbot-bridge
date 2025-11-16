"""
BonicBot Bridge - Python SDK for educational robotics programming
Provides high-level API for controlling BonicBot via ROS2 rosbridge
"""

from .core import BonicBot
from .exceptions import BonicBotError, ConnectionError, NavigationError

__version__ = "0.1.0"
__author__ = "Autobonics Pvt Ltd"

__all__ = ["BonicBot", "BonicBotError", "ConnectionError", "NavigationError"]