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


# Backward-compatible alias (avoid shadowing Python's built-in SystemError)
SystemError = SystemControlError
