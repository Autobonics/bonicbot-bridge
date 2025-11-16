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

class SystemError(BonicBotError):
    """Raised when system operations (mapping, navigation) fail"""
    pass