"""
Standalone Autonomous Exploration Controller

Architecture: This module talks directly to native ROS 2 services via roslibpy.
It does NOT import from system.py or motion.py to avoid their known reliability
issues (e.g. system.start_navigation() silently blocking when mapping is active).
"""

import time
import threading

import roslibpy
from roslibpy import Topic, Service, ServiceRequest
from .exceptions import BonicBotError
from .utils import (
    BOOL_MESSAGE_TYPE,
    STRING_MESSAGE_TYPE,
    TRIGGER_SERVICE_TYPE,
    OCCUPANCY_GRID_MESSAGE_TYPE,
    MARKER_ARRAY_MESSAGE_TYPE,
    GOAL_STATUS_ARRAY_MESSAGE_TYPE,
    MAP_TOPIC,
    EXPLORE_STATUS_TOPIC,
    EXPLORE_FRONTIERS_TOPIC,
    NAV2_COSTMAP_TOPIC,
    START_MAPPING_SERVICE,
    START_NAVIGATION_SERVICE,
    START_EXPLORE_SERVICE,
    safe_unsubscribe,
    safe_unadvertise,
)

class ExploreError(BonicBotError):
    """General exploration failure (publish error, ROS stack not ready)"""
    pass

class ExploreTimeoutError(ExploreError):
    """wait_for_map_complete() exhausts timeout without reaching min_area"""
    pass

MAP_THROTTLE_MS         = 2000   # 0.5 Hz — OccupancyGrid is large; never flood the bridge
FRONTIER_THROTTLE_MS    = 1000   # 1 Hz
VERIFY_TIMEOUT_S        = 10.0   # max wait for ROS stack verification in setup (/map check)
NAV2_COSTMAP_VERIFY_TIMEOUT_S = 30.0  # Bug 3: separate, longer timeout for costmap verification
NAV2_BOOT_TIMEOUT_S     = 70.0   # Nav2 takes ~60s to fully boot; give it headroom
DEFAULT_MAP_TIMEOUT_S   = 300.0  # 5 minutes


class ExploreController:
    def __init__(self, ros_client, system_controller=None):
        """
        Args:
            ros_client: connected roslibpy Ros instance (from bot.ros)
            system_controller: optional SystemController reference for state sync.
                               Never used for service calls — only to keep
                               navigation_active in sync during exploration.
        """
        self.ros = ros_client
        self._system = system_controller  # Bug 6: stored for state sync only

        # State to initialise
        self._state_lock = threading.Lock()  # Bug 7: guards shared state below
        self._explore_active = False
        self._latest_area_m2 = 0.0
        self._frontiers_received_once = False  # Bug 1: guard against false exhaustion
        self._frontiers_exhausted = False
        self._shutdown_called = False

        # Publisher (created once, advertised immediately)
        self._resume_pub = Topic(self.ros, EXPLORE_STATUS_TOPIC, BOOL_MESSAGE_TYPE)
        self._resume_pub.advertise()

        # Live background subscribers (throttled)
        self._map_sub = Topic(
            self.ros,
            MAP_TOPIC,
            OCCUPANCY_GRID_MESSAGE_TYPE,
            throttle_rate=MAP_THROTTLE_MS,
        )
        self._map_sub.subscribe(self._on_map)

        self._frontier_sub = Topic(
            self.ros,
            EXPLORE_FRONTIERS_TOPIC,
            MARKER_ARRAY_MESSAGE_TYPE,
            throttle_rate=FRONTIER_THROTTLE_MS,
        )
        self._frontier_sub.subscribe(self._on_frontiers)

    # ── Internal: Direct ROS Service Calls ───────────────────────────────────

    def _call_trigger(self, service_name, timeout):
        """Call a std_srvs/Trigger service directly via roslibpy.
        Returns the response dict. Raises ExploreError on failure."""
        srv = Service(self.ros, service_name, TRIGGER_SERVICE_TYPE)
        try:
            response = srv.call(ServiceRequest(), timeout=timeout)
        except Exception as exc:
            raise ExploreError(f"Service {service_name} call failed: {exc}")

        msg = response.get('message', '')
        if not response.get('success', False):
            # "already active" is not a failure — the subsystem is already running
            if 'already active' in msg.lower():
                print(f"ℹ️  {service_name}: {msg} (OK, continuing)")
                return response
            raise ExploreError(f"Service {service_name} returned failure: {msg}")
        return response

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_map(self, msg):
        """Callback — OccupancyGrid parser"""
        try:
            info = msg.get('info', {})
            resolution = info.get('resolution', 0.05)  # metres per cell (e.g. 0.05)
            data = msg.get('data', [])                 # flat int8 array, -1=unknown, 0=free, 100=occupied
            
            # Count known cells (!= -1)
            known_cells = sum(1 for cell in data if cell != -1)
            with self._state_lock:  # Bug 7
                self._latest_area_m2 = known_cells * (resolution ** 2)
        except Exception as exc:
            # Bug 4: Retain last good value; do not reset to 0.0
            print(f"⚠️ Error parsing /map (retaining last area {self._latest_area_m2:.1f}m²): {exc}")

    def _on_frontiers(self, msg):
        """Callback — frontier exhaustion detector"""
        markers = msg.get('markers', [])
        with self._state_lock:  # Bug 7
            if not self._explore_active:
                return
            # Bug 1: Only set exhausted after we've received at least one
            # non-empty frontier set, avoiding false positives during
            # explore_lite's initialization window.
            if len(markers) > 0:
                self._frontiers_received_once = True
            elif self._frontiers_received_once:
                self._frontiers_exhausted = True
                print("🗺️ explore: all frontiers exhausted — map complete")

    def start_explore(self) -> bool:
        """
        Publish {'data': True} to /explore/resume.
        
        Returns:
            bool: True on success.
            
        Raises:
            ExploreError: on publish failure.
        """
        try:
            self._resume_pub.publish({'data': True})
            with self._state_lock:  # Bug 7
                self._explore_active = True
                self._frontiers_received_once = False  # Bug 1: reset on start
                self._frontiers_exhausted = False
            print("🗺️ Exploration started")
            return True
        except Exception as exc:
            raise ExploreError(f"Failed to start exploration: {exc}")

    def stop_explore(self) -> bool:
        """
        Publish {'data': False} to /explore/resume.
        
        Returns:
            bool: True on success.
            
        Raises:
            ExploreError: on publish failure.
        """
        try:
            self._resume_pub.publish({'data': False})
            with self._state_lock:  # Bug 7
                self._explore_active = False
            print("🛑 Exploration stopped")
            return True
        except Exception as exc:
            raise ExploreError(f"Failed to stop exploration: {exc}")

    def is_exploring(self) -> bool:
        """
        Return self._explore_active; never blocks.
        """
        return self._explore_active

    def get_explored_area(self) -> float:
        """
        Return self._latest_area_m2; updated by background /map subscriber.
        """
        return self._latest_area_m2

    def setup_for_exploration(self, progress_callback=None) -> bool:
        """
        Boot SLAM + Nav2 + explore_lite via direct ROS service calls,
        then verify the data topics are alive before returning.
        
        This method bypasses system.py wrappers entirely because:
        - system.start_navigation() silently returns False when mapping is active
        - Exploration REQUIRES both mapping AND navigation to run concurrently
        - The robot_manager ROS node fully supports this (it checks mapping_active
          and launches nav2 in SLAM mode when true)
        
        Args:
            progress_callback: optional callable(step: int, total: int, stage: str, message: str)
                               Called at each setup stage for live progress reporting.
        
        Returns:
            bool: True if all subsystems verified.
            
        Raises:
            ExploreError: if any subsystem fails to start or verify.
        """
        TOTAL_STEPS = 6

        def _report(step, stage, message):
            print(message)
            if progress_callback:
                progress_callback(step, TOTAL_STEPS, stage, message)

        _report(0, 'init', '🔧 Setting up robot for exploration...')
        
        # Step 1: Start SLAM mapping (10s timeout — fast service)
        _report(1, 'slam', '⏳ Starting SLAM mapping...')
        self._call_trigger(START_MAPPING_SERVICE, timeout=15.0)
        _report(1, 'slam_done', '✅ Mapping service responded OK')

        # Step 2: Start Nav2 (70s timeout — Nav2 waits for its action server internally)
        _report(2, 'nav2', '⏳ Starting Nav2 (this takes ~60s on first boot)...')
        self._call_trigger(START_NAVIGATION_SERVICE, timeout=NAV2_BOOT_TIMEOUT_S)
        _report(2, 'nav2_done', '✅ Navigation service responded OK')

        # Step 3: Start explore_lite via the robot_manager service
        _report(3, 'explore_lite', '⏳ Starting explore_lite...')
        self._call_trigger(START_EXPLORE_SERVICE, timeout=15.0)
        _report(3, 'explore_lite_done', '✅ explore_lite service responded OK')

        # Step 4: Verify /map is actually publishing data
        map_event = threading.Event()
        def _map_verify_cb(msg):
            map_event.set()

        temp_map_sub = Topic(self.ros, MAP_TOPIC, OCCUPANCY_GRID_MESSAGE_TYPE)
        temp_map_sub.subscribe(_map_verify_cb)
        
        _report(4, 'verify_map', '⏳ Verifying SLAM (/map) data stream...')
        if not map_event.wait(timeout=VERIFY_TIMEOUT_S):
            safe_unsubscribe(temp_map_sub)
            raise ExploreError("SLAM not publishing /map within timeout")
        safe_unsubscribe(temp_map_sub)
        _report(4, 'verify_map_done', '✅ SLAM (/map) verified')

        # Step 5: Verify Nav2 is alive by checking costmap
        nav_event = threading.Event()
        def _nav_verify_cb(msg):
            nav_event.set()

        temp_nav_sub = Topic(self.ros, NAV2_COSTMAP_TOPIC, OCCUPANCY_GRID_MESSAGE_TYPE)
        temp_nav_sub.subscribe(_nav_verify_cb)
        
        _report(5, 'verify_nav', '⏳ Verifying Nav2 status stream...')
        if not nav_event.wait(timeout=NAV2_COSTMAP_VERIFY_TIMEOUT_S):  # Bug 3: longer timeout
            safe_unsubscribe(temp_nav_sub)
            raise ExploreError("Nav2 not publishing costmap within timeout")
        safe_unsubscribe(temp_nav_sub)
        _report(5, 'verify_nav_done', '✅ Nav2 verified')

        # Bug 2: Publish /explore/resume True + set flags via start_explore()
        # instead of setting flags directly (explore_lite needs the resume
        # topic to begin sending navigation goals).
        self.start_explore()

        # Bug 5: Sync navigation_active back to SystemController so that
        # code checking system.is_navigating() sees the correct state.
        if self._system is not None:
            self._system.navigation_active = True
            if self._system._motion is not None:
                self._system._motion._set_navigation_active(True)

        _report(6, 'complete', '✅ Exploration stack fully ready — robot is exploring!')
        return True

    def wait_for_map_complete(self, min_area: float, timeout: float = DEFAULT_MAP_TIMEOUT_S,
                              progress_callback=None) -> bool:
        """
        Poll _latest_area_m2 and _frontiers_exhausted.
        
        Args:
            min_area: target area in m² to consider the map "complete".
            timeout: maximum wait time in seconds.
            progress_callback: optional callable(current_area, min_area, elapsed, exhausted)
                               Called on every poll iteration for live progress.
        
        Returns:
            bool: True on either condition.
            
        Raises:
            ExploreTimeoutError: on timeout.
        """
        start = time.time()
        print(f"⏳ Waiting for map to complete (target: {min_area:.1f}m²)...")
        
        try:
            while True:
                elapsed = time.time() - start

                # Bug 7: snapshot shared state under lock
                with self._state_lock:
                    current_area = self._latest_area_m2
                    exhausted = self._frontiers_exhausted

                # Report progress
                if progress_callback:
                    progress_callback(current_area, min_area, elapsed, exhausted)
                
                # Exit condition 1: self._latest_area_m2 >= min_area
                if current_area >= min_area:
                    print(f"✅ Map complete: {current_area:.1f}m² reached target {min_area:.1f}m²")
                    return True
                    
                # Exit condition 2: self._frontiers_exhausted == True
                if exhausted:
                    print(f"✅ Map complete: all frontiers exhausted at {current_area:.1f}m²")
                    return True
                    
                # Timeout check
                if elapsed >= timeout:
                    raise ExploreTimeoutError(
                        f"map incomplete after {elapsed:.0f}s — "
                        f"area {current_area:.1f}m² / {min_area:.1f}m² target"
                    )
                    
                # Diagnostics on every poll iteration
                diag = self.diagnostics()
                print(f"🗺️ Explored: {current_area:.1f}m² / {min_area:.1f}m² — "
                      f"{elapsed:.0f}s elapsed | diag={diag}")
                time.sleep(2.0)  # Polling interval matches map throttle
        finally:
            self.stop_explore()

    def diagnostics(self) -> dict:
        """Return a snapshot of exploration state for debugging."""
        with self._state_lock:
            return {
                "explore_active": self._explore_active,
                "latest_area_m2": round(self._latest_area_m2, 2),
                "frontiers_received_once": self._frontiers_received_once,
                "frontiers_exhausted": self._frontiers_exhausted,
                "shutdown_called": self._shutdown_called,
            }

    def shutdown(self):
        """Teardown exploration controller safely."""
        if self._shutdown_called:
            return
        self._shutdown_called = True
        
        try:
            self.stop_explore()
        except ExploreError:
            pass  # Best effort
            
        safe_unsubscribe(self._map_sub)
        safe_unsubscribe(self._frontier_sub)
        safe_unadvertise(self._resume_pub)
