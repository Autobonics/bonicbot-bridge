"""
Standalone Autonomous Exploration Controller

Architecture: This module talks directly to native ROS 2 services via roslibpy.
It does NOT import from system.py or motion.py to avoid their known reliability
issues (e.g. system.start_navigation() silently blocking when mapping is active).

explore_lite integration:
  - Subscribes to /explore/status (ExploreStatus) for authoritative lifecycle events
  - Publishes to /explore/resume (std_msgs/Bool) to pause/resume
  - Calls /robot/start_explore and /robot/stop_explore services on robot_manager
  - Subscribes to /map for area calculation (same SLAM map explore_lite uses)
  - Subscribes to /explore/frontiers for visualization (secondary signal only)
"""

import time
import threading


from roslibpy import Topic, Service, ServiceRequest
from .exceptions import BonicBotError
from .utils import (
    BOOL_MESSAGE_TYPE,
    TRIGGER_SERVICE_TYPE,
    OCCUPANCY_GRID_MESSAGE_TYPE,
    MARKER_ARRAY_MESSAGE_TYPE,
    MAP_TOPIC,
    EXPLORE_RESUME_TOPIC,
    EXPLORE_LIFECYCLE_TOPIC,
    EXPLORE_STATUS_MESSAGE_TYPE,
    EXPLORE_FRONTIERS_TOPIC,
    NAV2_COSTMAP_TOPIC,
    START_MAPPING_SERVICE,
    START_NAVIGATION_SERVICE,
    STOP_NAVIGATION_SERVICE,
    START_EXPLORE_SERVICE,
    STOP_EXPLORE_SERVICE,
    EXPLORE_STATUS_COMPLETE,
    EXPLORE_STATUS_RETURNING,
    EXPLORE_STATUS_RETURNED,
    EXPLORE_STATUS_PAUSED,
    EXPLORE_STATUS_IN_PROGRESS,
    EXPLORE_STATUS_STARTED,
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
NAV2_COSTMAP_VERIFY_TIMEOUT_S = 30.0  # separate, longer timeout for costmap verification
NAV2_BOOT_TIMEOUT_S     = 70.0   # Nav2 takes ~60s to fully boot; give it headroom
DEFAULT_MAP_TIMEOUT_S   = 300.0  # 5 minutes

# Root-cause fix constants
COSTMAP_SETTLE_S        = 5.0    # wait after Nav2 verify before start_explore
MIN_EXPLORE_TIME_S      = 120.0  # minimum seconds before frontier exhaustion accepted
COSTMAP_KNOWN_CELL_MIN  = 50     # min known cells to consider costmap valid
NAV2_RESTART_WAIT_S     = 3.0    # wait after stop_navigation before restart


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
        self._system = system_controller  # stored for state sync only

        # State to initialise
        self._state_lock = threading.Lock()  # guards shared state below
        self._explore_active = False
        self._latest_area_m2 = 0.0
        self._frontiers_received_once = False  # guard against false exhaustion
        self._frontiers_exhausted = False
        self._shutdown_called = False
        self._explore_start_time = 0.0  # RC4: timestamp when exploration started

        self._area_before_goal = 0.0       # area snapshot when goal sent
        self._area_stagnant_count = 0      # consecutive goals with no growth

        # explore_lite lifecycle state (from /explore/status)
        self._explore_lifecycle = ""          # raw string from ExploreStatus.msg
        self._returned_to_init = False        # True after RETURNED_TO_ORIGIN

        # Lifecycle callback (set by dashboard to forward events)
        self._lifecycle_callback = None

        # Publisher: /explore/resume — the ACTUAL topic explore_lite subscribes to
        # for pause/resume control (NOT /robot/explore_active which is read-only)
        self._resume_pub = Topic(self.ros, EXPLORE_RESUME_TOPIC, BOOL_MESSAGE_TYPE)
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

        # Subscribe to /explore/status — authoritative lifecycle events from explore_lite
        self._status_sub = Topic(
            self.ros,
            EXPLORE_LIFECYCLE_TOPIC,
            EXPLORE_STATUS_MESSAGE_TYPE,
        )
        self._status_sub.subscribe(self._on_explore_status)

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
            with self._state_lock:
                self._latest_area_m2 = known_cells * (resolution ** 2)
        except Exception as exc:
            # Retain last good value; do not reset to 0.0
            print(f"⚠️ Error parsing /map (retaining last area {self._latest_area_m2:.1f}m²): {exc}")

    def _on_frontiers(self, msg):
        """Callback — frontier visualization marker tracker (secondary signal).
        
        The primary exhaustion signal comes from /explore/status (ExploreStatus).
        This callback tracks whether frontiers have ever been received, and
        enforces a minimum exploration time before accepting exhaustion.
        """
        markers = msg.get('markers', [])
        with self._state_lock:
            if not self._explore_active:
                return
            # RC4: Track that we've seen at least one non-empty frontier set
            if len(markers) > 0:
                self._frontiers_received_once = True
            elif self._frontiers_received_once:
                # RC4: Enforce minimum exploration time before accepting exhaustion
                elapsed = time.time() - self._explore_start_time
                if elapsed >= MIN_EXPLORE_TIME_S:
                    self._frontiers_exhausted = True
                    print("🗺️ explore: all frontiers exhausted — map complete")
                else:
                    remaining = MIN_EXPLORE_TIME_S - elapsed
                    print(f"⏳ explore: frontiers empty but minimum time not met "
                          f"({elapsed:.0f}s / {MIN_EXPLORE_TIME_S:.0f}s) — "
                          f"continuing for {remaining:.0f}s more")

    def _on_explore_status(self, msg):
        """Callback — authoritative lifecycle events from explore_lite node.
        
        explore_lite publishes ExploreStatus messages on /explore/status with
        status values: exploration_started, exploration_in_progress,
        exploration_paused, exploration_complete, returning_to_origin,
        returned_to_origin.
        
        RC4: exploration_complete is also gated by MIN_EXPLORE_TIME_S to prevent
        premature exhaustion from stale costmap data during the initial boot window.
        """
        status = msg.get('status', '')
        with self._state_lock:
            self._explore_lifecycle = status

            if status == EXPLORE_STATUS_COMPLETE:
                # RC4: Gate by minimum exploration time
                elapsed = time.time() - self._explore_start_time
                if elapsed >= MIN_EXPLORE_TIME_S:
                    self._frontiers_exhausted = True
                    print("🗺️ explore_lite: all frontiers exhausted — exploration complete")
                else:
                    remaining = MIN_EXPLORE_TIME_S - elapsed
                    print(f"⏳ explore_lite: reports complete but minimum time not met "
                          f"({elapsed:.0f}s / {MIN_EXPLORE_TIME_S:.0f}s) — "
                          f"ignoring for {remaining:.0f}s more")
            elif status == EXPLORE_STATUS_RETURNING:
                print("🗺️ explore_lite: returning to initial pose...")
            elif status == EXPLORE_STATUS_RETURNED:
                self._returned_to_init = True
                # RC4: Return-to-init also gated
                elapsed = time.time() - self._explore_start_time
                if elapsed >= MIN_EXPLORE_TIME_S:
                    self._frontiers_exhausted = True
                    print("🗺️ explore_lite: returned to initial pose — map complete")
                else:
                    print(f"⏳ explore_lite: returned to init but minimum time not met "
                          f"({elapsed:.0f}s / {MIN_EXPLORE_TIME_S:.0f}s)")
            elif status == EXPLORE_STATUS_PAUSED:
                print("🗺️ explore_lite: exploration paused")
            elif status in (EXPLORE_STATUS_STARTED, EXPLORE_STATUS_IN_PROGRESS):
                print(f"🗺️ explore_lite: {status}")

        # Fire dashboard callback if registered
        if self._lifecycle_callback:
            try:
                self._lifecycle_callback(status)
            except Exception as exc:
                print(f"⚠️ Lifecycle callback error: {exc}")

    def set_lifecycle_callback(self, callback):
        """Register a callback for explore_lite lifecycle events.
        
        Args:
            callback: callable(status: str) — called with the raw status string
                      from ExploreStatus.msg on every lifecycle change.
        """
        self._lifecycle_callback = callback

    def start_explore(self) -> bool:
        """
        Publish {'data': True} to /explore/resume to resume explore_lite.
        
        Returns:
            bool: True on success.
            
        Raises:
            ExploreError: on publish failure.
        """
        try:
            self._resume_pub.publish({'data': True})
            with self._state_lock:
                self._explore_active = True
                self._explore_start_time = time.time()  # RC4: record start time
                self._frontiers_received_once = False
                self._frontiers_exhausted = False
                self._returned_to_init = False
                self._explore_lifecycle = ""
            print("🗺️ Exploration started (published resume=True)")
            return True
        except Exception as exc:
            raise ExploreError(f"Failed to start exploration: {exc}")

    def stop_explore(self) -> bool:
        """
        Stop exploration: publish resume=False AND call /robot/stop_explore service
        to actually kill the explore_lite process on robot_manager.
        
        Returns:
            bool: True on success.
            
        Raises:
            ExploreError: on publish failure.
        """
        try:
            # 1. Pause explore_lite via its resume topic
            self._resume_pub.publish({'data': False})
            print("🛑 Exploration paused (published resume=False)")
        except Exception as exc:
            print(f"⚠️ Failed to publish resume=False: {exc}")

        try:
            # 2. Kill the explore_lite process via robot_manager service
            self._call_trigger(STOP_EXPLORE_SERVICE, timeout=10.0)
            print("🛑 explore_lite process stopped via /robot/stop_explore")
        except ExploreError as exc:
            # "not active" is fine — process may already be dead
            if 'not active' in str(exc).lower():
                print("ℹ️ explore_lite was already stopped")
            else:
                print(f"⚠️ Failed to stop explore_lite process: {exc}")

        with self._state_lock:
            self._explore_active = False
        return True

    def suspend_for_manual_control(self) -> bool:
        """
        Pause exploration and stop Nav2 to allow manual joystick control.
        Call resume_from_manual_control() to restart.

        Returns:
            bool: True if both operations succeeded.
        """
        try:
            self.stop_explore()
            if self._system:
                self._system.stop_navigation()
                print("⏸️ Exploration suspended — joystick now controls /cmd_vel")
            return True
        except Exception as exc:
            raise ExploreError(f"Failed to suspend for manual control: {exc}")

    def resume_from_manual_control(self) -> bool:
        """
        Restart Nav2 and resume exploration after manual driving.
        Does NOT call setup_for_exploration() — assumes SLAM already running.
        """
        try:
            if self._system:
                self._system.start_navigation()
            self.start_explore()
            print("▶️ Exploration resumed from manual control")
            return True
        except Exception as exc:
            raise ExploreError(f"Failed to resume from manual control: {exc}")

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
        TOTAL_STEPS = 7

        def _report(step, stage, message):
            print(message)
            if progress_callback:
                progress_callback(step, TOTAL_STEPS, stage, message)

        _report(0, 'init', '🔧 Setting up robot for exploration...')
        
        # Step 1: Start SLAM mapping (10s timeout — fast service)
        _report(1, 'slam', '⏳ Starting SLAM mapping...')
        self._call_trigger(START_MAPPING_SERVICE, timeout=15.0)
        _report(1, 'slam_done', '✅ Mapping service responded OK')

        # Step 2: Start Nav2 — RC1: force restart if "already active" to clear stale costmap
        _report(2, 'nav2', '⏳ Starting Nav2 (this takes ~60s on first boot)...')
        nav_response = self._call_trigger(START_NAVIGATION_SERVICE, timeout=NAV2_BOOT_TIMEOUT_S)
        nav_msg = nav_response.get('message', '').lower()
        if 'already active' in nav_msg:
            _report(2, 'nav2_restart', '⚠️ Nav2 was already active with stale costmap — restarting fresh...')
            self._call_trigger(STOP_NAVIGATION_SERVICE, timeout=15.0)
            time.sleep(NAV2_RESTART_WAIT_S)
            self._call_trigger(START_NAVIGATION_SERVICE, timeout=NAV2_BOOT_TIMEOUT_S)
            _report(2, 'nav2_done', '✅ Navigation restarted fresh')
        else:
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

        # Step 5: RC2 — Verify Nav2 costmap has real data (not all-unknown)
        costmap_ready = threading.Event()
        def _nav_verify_cb(msg):
            data = msg.get('data', [])
            # Accept costmap only when it has known cells (not all -1/unknown)
            known = sum(1 for c in data if c != -1)
            if known >= COSTMAP_KNOWN_CELL_MIN:
                costmap_ready.set()

        temp_nav_sub = Topic(self.ros, NAV2_COSTMAP_TOPIC, OCCUPANCY_GRID_MESSAGE_TYPE)
        temp_nav_sub.subscribe(_nav_verify_cb)
        
        _report(5, 'verify_nav', '⏳ Verifying Nav2 costmap has known cells...')
        if not costmap_ready.wait(timeout=NAV2_COSTMAP_VERIFY_TIMEOUT_S):
            safe_unsubscribe(temp_nav_sub)
            raise ExploreError("Nav2 costmap has no known cells within timeout")
        safe_unsubscribe(temp_nav_sub)
        _report(5, 'verify_nav_done', '✅ Nav2 costmap verified with known cells')

        # Step 6: RC3 — Settle delay for costmap to fully inflate current SLAM map
        _report(6, 'settling', f'⏳ Waiting {COSTMAP_SETTLE_S}s for costmap to settle...')
        time.sleep(COSTMAP_SETTLE_S)
        _report(6, 'settle_done', '✅ Costmap settled')

        # Step 7: Publish /explore/resume True + set flags via start_explore()
        # explore_lite needs the resume topic to begin sending navigation goals.
        self.start_explore()

        # Sync navigation_active back to SystemController so that
        # code checking system.is_navigating() sees the correct state.
        if self._system is not None:
            self._system.navigation_active = True
            if self._system._motion is not None:
                self._system._motion._set_navigation_active(True)

        _report(7, 'complete', '✅ Exploration stack fully ready — robot is exploring!')
        return True

    def _check_area_growth(self, area_before: float, area_after: float,
                            goal_index: int) -> None:
        """Log warning if a completed navigation goal produced no map growth."""
        growth = area_after - area_before
        AREA_GROWTH_MIN_M2 = 0.1   # expect at least 0.1m² new area per goal
        if growth < AREA_GROWTH_MIN_M2:
            self._area_stagnant_count += 1
            print(
                f"⚠️ Goal {goal_index} completed but map grew only "
                f"{growth:.2f}m² (< {AREA_GROWTH_MIN_M2}m²). "
                f"Stagnant count: {self._area_stagnant_count}. "
                f"Possible causes: LIDAR range too short, Nav2 blocked "
                f"joystick during drive, or min_frontier_size too large."
            )
        else:
            self._area_stagnant_count = 0
            print(f"✅ Goal {goal_index}: map grew {growth:.2f}m² "
                  f"(total: {area_after:.1f}m²)")

    def wait_for_map_complete(self, min_area: float, timeout: float = DEFAULT_MAP_TIMEOUT_S,
                              progress_callback=None) -> bool:
        """
        Poll _latest_area_m2 and _frontiers_exhausted.
        
        Completion is detected via two signals:
          1. Area threshold reached: _latest_area_m2 >= min_area
          2. explore_lite reports EXPLORATION_COMPLETE or RETURNED_TO_ORIGIN
             via /explore/status (authoritative, gated by MIN_EXPLORE_TIME_S)
        
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

                # Snapshot shared state under lock
                with self._state_lock:
                    current_area = self._latest_area_m2
                    exhausted = self._frontiers_exhausted
                    lifecycle = self._explore_lifecycle

                if hasattr(self, '_last_polled_area'):
                    delta = current_area - self._last_polled_area
                    if delta > 0.05:
                        print(f"📈 Map growing: +{delta:.2f}m² this cycle")
                    elif self._explore_active and elapsed > 60:
                        print(f"⚠️ Map stagnant for this cycle at {current_area:.1f}m² "
                              f"— check LIDAR and cmd_vel arbitration")
                self._last_polled_area = current_area

                # Report progress
                if progress_callback:
                    progress_callback(current_area, min_area, elapsed, exhausted)
                
                # Exit condition 1: area threshold met
                if current_area >= min_area:
                    print(f"✅ Map complete: {current_area:.1f}m² reached target {min_area:.1f}m²")
                    return True
                    
                # Exit condition 2: explore_lite reports exploration complete
                # Secondary fix: log whether area target was actually met
                if exhausted:
                    if current_area >= min_area:
                        print(f"✅ Map complete: frontiers exhausted + area target met "
                              f"({current_area:.1f}m² / {min_area:.1f}m²)")
                    else:
                        print(f"⚠️ Frontiers exhausted but area {current_area:.1f}m² "
                              f"< target {min_area:.1f}m² — map may be incomplete")
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
                "explore_lifecycle": self._explore_lifecycle,
                "returned_to_init": self._returned_to_init,
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
        safe_unsubscribe(self._status_sub)
        safe_unadvertise(self._resume_pub)
