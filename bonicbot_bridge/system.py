"""
System controller for high-level robot operations
"""

from roslibpy import Topic, Service, ServiceRequest
from .exceptions import BonicSystemError

class SystemController:
    def __init__(self, ros_client):
        self.ros = ros_client
        
        # System status topics
        self.state_sub = Topic(self.ros, '/robot/state', 'std_msgs/String')
        self.mapping_status_sub = Topic(self.ros, '/robot/mapping_active', 'std_msgs/Bool')
        self.nav_status_sub = Topic(self.ros, '/robot/navigation_active', 'std_msgs/Bool')
        self.camera_status_sub = Topic(self.ros, '/robot/camera_active', 'std_msgs/Bool')
        
        # System control services
        self.start_mapping_srv = Service(self.ros, '/robot/start_mapping', 'std_srvs/Trigger')
        self.stop_mapping_srv = Service(self.ros, '/robot/stop_mapping', 'std_srvs/Trigger')
        self.save_map_srv = Service(self.ros, '/robot/save_map', 'std_srvs/Trigger')
        self.start_nav_srv = Service(self.ros, '/robot/start_navigation', 'std_srvs/Trigger')
        self.stop_nav_srv = Service(self.ros, '/robot/stop_navigation', 'std_srvs/Trigger')
        self.start_camera_srv = Service(self.ros, '/robot/start_camera', 'std_srvs/Trigger')
        self.stop_camera_srv = Service(self.ros, '/robot/stop_camera', 'std_srvs/Trigger')
        
        # System state
        self.robot_state = 'idle'
        self.mapping_active = False
        self.navigation_active = False
        self.camera_active = False
        
        # Subscribe to status updates
        self.state_sub.subscribe(self._state_callback)
        self.mapping_status_sub.subscribe(self._mapping_callback)
        self.nav_status_sub.subscribe(self._nav_callback)
        self.camera_status_sub.subscribe(self._camera_callback)
    
    def _state_callback(self, msg):
        """Update robot state"""
        self.robot_state = msg['data']
        
    def _mapping_callback(self, msg):
        """Update mapping status"""
        self.mapping_active = msg['data']
        
    def _nav_callback(self, msg):
        """Update navigation status"""
        self.navigation_active = msg['data']
    
    def _camera_callback(self, msg):
        """Update camera status"""
        self.camera_active = msg['data']
    
    def start_mapping(self):
        """
        Start SLAM mapping mode.

        This method is idempotent — calling it when mapping is
        already active is a safe no-op.
        
        Returns:
            bool: True if mapping started (or was already active)
        """
        if self.mapping_active:
            print("ℹ️ Mapping is already active, nothing to start")
            return True

        try:
            request = ServiceRequest()
            response = self.start_mapping_srv.call(request, timeout=30)
            
            if not response['success']:
                raise BonicSystemError(f"Failed to start mapping: {response['message']}")
            
            print("🗺️ Mapping started - robot will create a map as it moves")
            return True
            
        except Exception as e:
            raise BonicSystemError(f"Mapping start failed: {str(e)}")
    
    def stop_mapping(self):
        """
        Stop SLAM mapping mode
        
        Returns:
            bool: True if mapping stopped successfully
        """
        try:
            request = ServiceRequest()
            response = self.stop_mapping_srv.call(request, timeout=30)
            
            if not response['success']:
                raise BonicSystemError(f"Failed to stop mapping: {response['message']}")
            
            print("🛑 Mapping stopped")
            return True
            
        except Exception as e:
            raise BonicSystemError(f"Mapping stop failed: {str(e)}")
    
    def save_map(self):
        """
        Save the current map.
        Note: The map name is determined automatically by the backend system.
        
        Returns:
            bool: True if map saved successfully
        """
        try:
            request = ServiceRequest()
            response = self.save_map_srv.call(request, timeout=30)
            
            if not response['success']:
                raise BonicSystemError(f"Failed to save map: {response['message']}")
            
            print(f"💾 Map saved successfully: {response['message']}")
            return True
            
        except Exception as e:
            raise BonicSystemError(f"Map save failed: {str(e)}")
    
    def start_navigation(self):
        """
        Start navigation mode (requires saved map or active mapping).

        This method is idempotent — calling it when navigation is
        already active is a safe no-op.
        
        Returns:
            bool: True if navigation started (or was already active)
        """
        if self.navigation_active:
            print("ℹ️ Navigation is already active, nothing to start")
            return True

        try:
            request = ServiceRequest()
            response = self.start_nav_srv.call(request, timeout=30)
            
            if not response['success']:
                raise BonicSystemError(f"Failed to start navigation: {response['message']}")
            
            print("🧭 Navigation started - robot can now navigate to goals")
            return True
            
        except Exception as e:
            raise BonicSystemError(f"Navigation start failed: {str(e)}")
    
    def stop_navigation(self):
        """
        Stop navigation mode
        
        Returns:
            bool: True if navigation stopped successfully
        """
        try:
            request = ServiceRequest()
            response = self.stop_nav_srv.call(request, timeout=30)
            
            if not response['success']:
                raise BonicSystemError(f"Failed to stop navigation: {response['message']}")
            
            print("🛑 Navigation stopped")
            return True
            
        except Exception as e:
            raise BonicSystemError(f"Navigation stop failed: {str(e)}")
    
    def get_system_status(self):
        """
        Get current system status
        
        Returns:
            dict: System status information
        """
        return {
            'state': self.robot_state,
            'mapping_active': self.mapping_active,
            'navigation_active': self.navigation_active,
            'camera_active': self.camera_active,
            'ready_for_goals': self.navigation_active
        }
    
    def is_mapping(self):
        """Check if robot is currently mapping"""
        return self.mapping_active
        
    def is_navigating(self):
        """Check if navigation system is active"""
        return self.navigation_active
        
    def get_robot_state(self):
        """Get current robot state string"""
        return self.robot_state
    
    def start_camera(self):
        """
        Start camera system
        
        Returns:
            bool: True if camera started successfully
        """
        try:
            request = ServiceRequest()
            response = self.start_camera_srv.call(request, timeout=30)
            
            if not response['success']:
                raise BonicSystemError(f"Failed to start camera: {response['message']}")
            
            print("📷 Camera started")
            return True
            
        except Exception as e:
            raise BonicSystemError(f"Camera start failed: {str(e)}")
    
    def stop_camera(self):
        """
        Stop camera system.

        This method is idempotent — calling it when the camera is
        already inactive is a safe no-op.
        
        Returns:
            bool: True if camera stopped (or was already stopped)
        """
        if not self.camera_active:
            print("ℹ️ Camera is already inactive, nothing to stop")
            return True

        try:
            request = ServiceRequest()
            response = self.stop_camera_srv.call(request, timeout=30)
            
            if not response['success']:
                raise BonicSystemError(f"Failed to stop camera: {response['message']}")
            
            print("🛑 Camera stopped")
            return True
            
        except Exception as e:
            raise BonicSystemError(f"Camera stop failed: {str(e)}")
    
    def is_camera_active(self):
        """Check if camera system is active"""
        return self.camera_active
    
    def setup_for_mapping(self):
        """
        Helper function to set up robot for mapping
        
        Returns:
            bool: True if setup successful
        """
        print("🔧 Setting up robot for mapping...")
        
        # Stop navigation if active
        if self.navigation_active:
            self.stop_navigation()
        
        # Start mapping
        return self.start_mapping()
    
    def setup_for_navigation(self):
        """
        Helper function to set up robot for autonomous navigation
        
        Returns:
            bool: True if setup successful
        """
        print("🔧 Setting up robot for navigation...")
        
        # Start navigation (will automatically check for saved map)
        return self.start_navigation()
    
    def quick_map_and_nav(self):
        """
        Helper function for simultaneous mapping and navigation.
        Useful for exploring unknown areas.

        This method is idempotent — subsystems that are already
        running are silently skipped.
        
        Returns:
            bool: True if both subsystems are active after the call
        """
        print("🔧 Starting mapping and navigation together...")
        
        # Start mapping (idempotent — no-op if already active)
        try:
            self.start_mapping()
        except BonicSystemError as e:
            print(f"⚠️ Could not start mapping: {e}")
            if not self.mapping_active:
                print("❌ Mapping is not active — aborting")
                return False
            # Mapping is active despite the error (race condition),
            # so we can safely continue.
            print("ℹ️ Mapping is active despite the error, continuing")
        
        # Start navigation (idempotent — no-op if already active)
        try:
            self.start_navigation()
        except BonicSystemError as e:
            print(f"⚠️ Could not start navigation: {e}")
            if not self.navigation_active:
                print("❌ Navigation is not active — aborting")
                return False
            print("ℹ️ Navigation is active despite the error, continuing")
            
        print("✅ Robot ready for exploration (mapping + navigation)")
        return True