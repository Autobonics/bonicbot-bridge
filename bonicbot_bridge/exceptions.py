"""
Custom exceptions for BonicBot Bridge
"""


class BonicBotError(Exception):
    """Base exception for BonicBot operations"""

    pass


class ConnectionError(BonicBotError):
    """Raised when connection to robot fails"""

    pass


class NavigationError(BonicBotError):
    """Raised when navigation operations fail"""

    pass


class PreciseMotionError(BonicBotError):
    """Raised when a precise motion command fails (distance guard, timeout, etc.)"""

    pass


class SystemControlError(BonicBotError):
    """Raised when system operations (mapping, navigation) fail"""

    pass


class VisionError(BonicBotError):
    """Raised when a vision pipeline operation fails"""

    pass


class ServoError(BonicBotError):
    """Raised when a servo/joint command fails"""

    pass


class CameraError(BonicBotError):
    """Raised when a camera operation fails"""

    pass


class SensorError(BonicBotError):
    """Raised when a sensor read or subscription fails"""

    pass


class ExploreError(BonicBotError):
    """Raised when autonomous exploration fails"""

    pass


class ExploreTimeoutError(ExploreError):
    """Raised when exploration setup or wait exceeds timeout"""

    pass


# Backward-compatible alias (avoid shadowing Python's built-in SystemError)
SystemError = SystemControlError
