"""
Thread-safe command queue mixin for MotionController.

Provides enqueue_move(), run_queue(), clear_queue(), and draw_square().
Initialised lazily via _init_queue() from the host class's __init__.

This module acts as a bridge to robot_manager.py, handling queue execution 
and Nav2 start/wait logic. It delegates actual motion to either the 
obstacle-aware Nav2 engine or the fast internal odometry loop.
"""

import enum
import json
import queue
import threading
import time
import traceback

from roslibpy import Service, ServiceRequest, Topic

from .exceptions import PreciseMotionError

class PreciseMotionPrematureCompletionError(PreciseMotionError):
    """Raised when a precise motion command completes suspiciously fast, indicating a failure."""
    pass
from .utils import (
    BOOL_MESSAGE_TYPE,
    MAX_PRECISE_DISTANCE,
    NAVIGATION_STATUS_TOPIC,
    START_NAVIGATION_SERVICE,
    STRING_MESSAGE_TYPE,
    TRIGGER_SERVICE_TYPE,
    safe_unsubscribe,
    safe_unadvertise,
)

# ── Queue command types ────────────────────────────────────────────────
CMD_TYPE_DRIVE = "drive"
CMD_TYPE_ROTATE = "rotate"

# ── Defaults ───────────────────────────────────────────────────────────
DEFAULT_QUEUE_DRIVE_SPEED = 0.3      # m/s
DEFAULT_QUEUE_ROTATE_SPEED = 45.0    # deg/s
DEFAULT_QUEUE_TIMEOUT = 30.0         # seconds per command
DEFAULT_QUEUE_ENGINE = "nav2"
QUEUE_POLL_INTERVAL_SECONDS = 0.05   # 50 ms between empty-queue checks

# ── Precise-motion constants ───────────────────────────────────────────

# Bridge communication topics (matching robot_manager.py)
PRECISE_MOVE_TOPIC = "/robot/precise_move"
PRECISE_MOVE_ACTIVE_TOPIC = "/robot/precise_move_active"

# Timing for wait-for-completion polling
PRECISE_MOVE_POLL_INTERVAL_SECONDS = 0.05  # 50 ms
PRECISE_MOVE_START_TIMEOUT_SECONDS = 10.0  # max wait for manager to acknowledge
NAV2_ACTIVATION_TIMEOUT_SECONDS = 30.0    # max wait for Nav2 to come up
NAV2_ACTIVATION_POLL_SECONDS = 0.1        # poll interval while waiting for Nav2

# Default speeds for drive methods
DEFAULT_DRIVE_SPEED = 0.3   # m/s
DEFAULT_ROTATE_SPEED = 45.0  # deg/s

# Default timeouts
DRIVE_DISTANCE_TIMEOUT_SECONDS = 30.0
ROTATE_ANGLE_TIMEOUT_SECONDS = 30.0

# Engine enum — use this for type-safe engine selection across the codebase
class PreciseMotionEngine(enum.Enum):
    """Execution engine for precise motion commands.

    Members:
        INTERNAL: Closed-loop control via /cmd_vel + odometry (obstacle-blind).
        NAV2: Nav2 actions (DriveOnHeading/Spin) with costmap collision checking.
    """
    INTERNAL = "internal"
    NAV2 = "nav2"


class QueueMixin:
    """Thread-safe command queue for sequential motion execution.
    Mix into MotionController. Host must call _init_queue() and _queue_shutdown().
    """

    # ── Initialisation (called from MotionController.__init__) ───────

    def _init_queue(self):
        """Initialise queue and precise motion bridge infrastructure.  Call once from __init__."""
        self._cmd_queue = queue.Queue()
        self._queue_cancel = threading.Event()
        self._queue_done_event = threading.Event()
        self._queue_thread = None
        self._queue_lock = threading.Lock()
        # Holds the boolean result of the most recent run_queue() execution
        self._queue_result = True

        # Bridge: publisher to send commands to robot_manager.py
        self._precise_move_pub = Topic(
            self.ros, PRECISE_MOVE_TOPIC, STRING_MESSAGE_TYPE
        )
        self._precise_move_pub.advertise()

        # Bridge: subscriber to track when robot_manager.py starts/finishes
        self._precise_active_sub = Topic(
            self.ros, PRECISE_MOVE_ACTIVE_TOPIC, BOOL_MESSAGE_TYPE
        )
        self._precise_active = False
        self._precise_active_lock = threading.Lock()
        self._precise_active_sub.subscribe(self._precise_active_callback)

        # Bridge: service client for starting Nav2 on-demand
        self._start_nav_srv = Service(
            self.ros, START_NAVIGATION_SERVICE, TRIGGER_SERVICE_TYPE
        )

        # Bridge: subscriber to track whether Nav2 is actually running
        self._nav_active_sub = Topic(
            self.ros, NAVIGATION_STATUS_TOPIC, BOOL_MESSAGE_TYPE
        )
        self._nav_active = False
        self._nav_active_lock = threading.Lock()
        self._nav_active_sub.subscribe(self._nav_active_callback)

        # Precise-motion infrastructure — default to NAV2 for obstacle avoidance
        self._default_engine = PreciseMotionEngine.NAV2
        self._precise_moving_flag = threading.Event()  # SET while executing
        self._nav2_settle_seconds = 30.0  # Time to wait for behavior servers after nav is active

    # ── Bridge callbacks ─────────────────────────────────────────────

    def _precise_active_callback(self, msg: dict) -> None:
        """Track the precise_move_active status published by robot_manager.py.

        Args:
            msg: roslibpy message dict with a ``data`` bool field.
        """
        with self._precise_active_lock:
            self._precise_active = msg["data"]

    def _nav_active_callback(self, msg: dict) -> None:
        """Track the navigation_active status published by robot_manager.py.

        Args:
            msg: roslibpy message dict with a ``data`` bool field.
        """
        with self._nav_active_lock:
            self._nav_active = msg["data"]

    # ── Configuration API ────────────────────────────────────────────

    def set_nav2_settle_time(self, seconds: float) -> None:
        """Set the settle delay used after Nav2 becomes active.

        Args:
            seconds: Float indicating seconds to wait for behavior servers
                     (e.g., DriveOnHeading) to be fully ready.
        """
        self._nav2_settle_seconds = float(seconds)

    # ── Public API ───────────────────────────────────────────────────

    def enqueue_move(self, cmd_list):
        """Push motion commands onto the queue (non-blocking).

        Args:
            cmd_list (list[dict]): List of commands. Each dict must have 'type' 
                                   ('drive'/'rotate') and 'value' (amount).
                                   Optional: 'speed', 'engine', 'timeout'.
        """
        for cmd in cmd_list:
            if cmd.get("type") not in (CMD_TYPE_DRIVE, CMD_TYPE_ROTATE):
                print(
                    f"⚠️ enqueue_move: unknown command type "
                    f"'{cmd.get('type')}' — skipping"
                )
                continue
            self._cmd_queue.put(cmd)

    def run_queue(self, block=True):
        """Start executing queued commands on a background thread.
        
        Args:
            block (bool): Block caller until completion if True.

        Returns:
            bool: True if all commands succeeded, False otherwise (always True if block=False).
        """
        self._queue_cancel.clear()
        self._queue_done_event.clear()
        self._queue_result = True

        with self._queue_lock:
            # Start a new worker only if there isn't one alive already
            if self._queue_thread is None or not self._queue_thread.is_alive():
                self._queue_thread = threading.Thread(
                    target=self._queue_worker,
                    name="bonicbot-queue-worker",
                    daemon=True,
                )
                self._queue_thread.start()

        if block:
            self._queue_done_event.wait()
            return self._queue_result

        # Non-blocking: we can't know the result yet
        return True

    def clear_queue(self):
        """Flush pending commands and abort the currently executing one.
        Safe to call from any thread.
        """
        # 1. Drain the queue
        while True:
            try:
                self._cmd_queue.get_nowait()
            except queue.Empty:
                break

        # 2. Tell the worker loop to stop
        self._queue_cancel.set()

        # 3. Cancel any in-progress drive_distance / rotate_angle
        #    (_move_cancel is defined in MotionController)
        self._move_cancel.set()

        # 4. Halt the robot immediately
        try:
            self.stop()
        except Exception:
            pass

        print("🛑 clear_queue: queue flushed and motion cancelled")

    def draw_square(self, side_m, speed=DEFAULT_QUEUE_DRIVE_SPEED,
                    turn_speed=DEFAULT_QUEUE_ROTATE_SPEED,
                    engine=DEFAULT_QUEUE_ENGINE,
                    timeout=DEFAULT_QUEUE_TIMEOUT):
        """Drive the robot in a square pattern.

        Enqueues 4x [drive side_m, rotate 90°] and executes sequentially.

        Returns:
            bool: True if full square completed, False if any leg failed.
        """
        cmds = []
        for _ in range(4):
            cmds.append({
                "type": CMD_TYPE_DRIVE,
                "value": side_m,
                "speed": speed,
                "engine": engine,
                "timeout": timeout,
            })
            cmds.append({
                "type": CMD_TYPE_ROTATE,
                "value": 90.0,
                "speed": turn_speed,
                "engine": engine,
                "timeout": timeout,
            })

        print(
            f"▶️  draw_square: {side_m:.2f}m sides, "
            f"{speed} m/s, {turn_speed} deg/s, engine={engine}"
        )

        self.enqueue_move(cmds)
        result = self.run_queue(block=True)

        if result:
            print("✅ draw_square: completed successfully")
        else:
            print("❌ draw_square: failed mid-execution")

        return result

    # ── Background worker ────────────────────────────────────────────

    def _queue_worker(self):
        """Daemon thread target — pull and execute commands one by one."""
        try:
            while not self._queue_cancel.is_set():
                # Non-blocking get so we can check the cancel flag
                try:
                    cmd = self._cmd_queue.get(block=True, timeout=QUEUE_POLL_INTERVAL_SECONDS)
                except queue.Empty:
                    # Queue drained — we're done successfully
                    break

                if self._queue_cancel.is_set():
                    break

                success = self._execute_single_command(cmd)

                if not success:
                    print(
                        f"❌ run_queue: command failed — "
                        f"type={cmd.get('type')}, value={cmd.get('value')}"
                    )
                    self._queue_result = False
                    self.clear_queue()
                    return  # finally block will still fire

        except Exception as exc:
            print(f"❌ run_queue: unexpected error in worker thread: {exc}")
            traceback.print_exc()
            self._queue_result = False
            self.clear_queue()
        finally:
            # Always ensure robot is stopped
            try:
                self.stop()
            except Exception:
                pass
            self._queue_done_event.set()

    def _execute_single_command(self, cmd):
        """Dispatch a single command dict to the appropriate motion method.

        Args:
            cmd (dict): Command dictionary with keys ``type``, ``value``,
                and optionally ``speed``, ``engine``, ``timeout``.

        Returns:
            bool: True if the command succeeded, False otherwise.
        """
        cmd_type = cmd["type"]
        value = cmd["value"]
        engine = cmd.get("engine", DEFAULT_QUEUE_ENGINE)
        timeout = cmd.get("timeout", DEFAULT_QUEUE_TIMEOUT)

        try:
            if cmd_type == CMD_TYPE_DRIVE:
                speed = cmd.get("speed", DEFAULT_QUEUE_DRIVE_SPEED)
                return self.drive_distance(
                    dist=value, speed=speed, engine=engine, timeout=timeout
                )
            elif cmd_type == CMD_TYPE_ROTATE:
                speed = cmd.get("speed", DEFAULT_QUEUE_ROTATE_SPEED)
                return self.rotate_angle(
                    angle=value, speed=speed, engine=engine, timeout=timeout
                )
            else:
                print(f"⚠️ _execute_single_command: unknown type '{cmd_type}'")
                return False

        except Exception as exc:
            print(f"❌ _execute_single_command: {cmd_type}({value}) error: {exc}")
            traceback.print_exc()
            return False

    # ── Precise-motion methods (Bridge to robot_manager.py) ───────────
    #
    # Public:  set_default_engine, is_precise_moving, drive_distance,
    #          rotate_angle, drive_and_rotate
    # Private: _resolve_engine, _wait_for_precise_move,
    #          _publish_precise_command

    def set_default_engine(self, engine):
        """Set the default execution engine for precise motion commands.

        The default engine is used when ``drive_distance()``,
        ``rotate_angle()``, or ``drive_and_rotate()`` are called without
        an explicit ``engine`` argument.

        Args:
            engine (PreciseMotionEngine): The engine to use by default.
                Must be a ``PreciseMotionEngine`` enum member.

        Raises:
            PreciseMotionError: If *engine* is not a ``PreciseMotionEngine`` member.

        Returns:
            None
        """
        if not isinstance(engine, PreciseMotionEngine):
            raise PreciseMotionError(
                f"engine must be a PreciseMotionEngine member, "
                f"got {type(engine).__name__}"
            )
        self._default_engine = engine

    def is_precise_moving(self):
        """Check if a precise motion command is currently executing.

        Returns ``True`` while any ``drive_distance`` / ``rotate_angle`` /
        ``drive_and_rotate`` command is actively running.  The flag is set
        before the command is published and cleared in a ``finally`` block,
        so it resets even if an exception is raised mid-motion.

        Returns:
            bool: ``True`` if a precise motion command is in progress.
        """
        return self._precise_moving_flag.is_set()

    def _resolve_engine(self, engine):
        """Normalise an engine parameter to a PreciseMotionEngine member.

        Accepts PreciseMotionEngine members, plain strings ('internal',
        'nav2'), or None (falls back to _default_engine).

        Args:
            engine: PreciseMotionEngine member, str, or None.

        Returns:
            PreciseMotionEngine: Resolved engine member.

        Raises:
            PreciseMotionError: If *engine* is an unrecognised string, or if
                it is not str, PreciseMotionEngine, or None.
        """
        if engine is None:
            return self._default_engine
        if isinstance(engine, PreciseMotionEngine):
            return engine
        if isinstance(engine, str):
            try:
                return PreciseMotionEngine(engine)
            except ValueError:
                raise PreciseMotionError(
                    f"Unknown engine '{engine}'. "
                    f"Valid engines: {[e.value for e in PreciseMotionEngine]}"
                )
        raise PreciseMotionError(
            f"engine must be PreciseMotionEngine, str, or None — "
            f"got {type(engine).__name__}"
        )

    def _publish_precise_command(self, payload: dict) -> None:
        """Publish a JSON command to /robot/precise_move for robot_manager.py.

        Args:
            payload: Command dictionary to JSON-serialise and publish.
        """
        msg = {"data": json.dumps(payload)}
        self._precise_move_pub.publish(msg)

    def _ensure_nav2_active(self) -> None:
        """Ensure Nav2 is running before sending a Nav2 precise-move command.

        If ``/robot/navigation_active`` is already True, returns immediately.
        Otherwise calls ``/robot/start_navigation`` and polls the subscription
        until navigation_active flips True, with a timeout.

        Raises:
            PreciseMotionError: If Nav2 cannot be activated within
                NAV2_ACTIVATION_TIMEOUT_SECONDS.
        """
        # Fast path: already running
        with self._nav_active_lock:
            if self._nav_active:
                return

        # Call the service to start Nav2
        print("🔄 Nav2 not active — calling /robot/start_navigation...")
        try:
            request = ServiceRequest()
            response = self._start_nav_srv.call(request, timeout=15)
            # start_navigation may return success=False if already active,
            # which is fine — we just need navigation_active to go True.
            print(f"   /robot/start_navigation response: {response.get('message', '')}")
            
            # Inject a default initial pose to ensure AMCL unblocks the Nav2 lifecycle.
            # We do this immediately after starting Nav2 so AMCL is running to receive it.
            # This is harmless if mapping is active or if AMCL already has a pose.
            try:
                if hasattr(self, "set_initial_pose"):
                    print("   Injecting default initial pose to unblock Nav2 lifecycle...")
                    self.set_initial_pose(0.0, 0.0, 0.0)
            except Exception as e:
                print(f"⚠️ Could not set default initial pose: {e}")
                
        except Exception as exc:
            raise PreciseMotionError(
                f"Failed to call /robot/start_navigation: {exc}"
            )

        # Poll until navigation_active goes True
        deadline = time.time() + NAV2_ACTIVATION_TIMEOUT_SECONDS
        while time.time() < deadline:
            with self._nav_active_lock:
                if self._nav_active:
                    print("✅ Nav2 is now active — waiting for behavior servers to settle...")
                    # navigation_active goes True when the nav2 subprocess starts,
                    # but individual servers (behavior_server, controller_server)
                    # need additional time to complete their lifecycle transition.
                    # Without this settle delay, robot_manager's
                    # wait_for_server(timeout_sec=5) on DriveOnHeading/Spin can
                    # timeout and silently drop the command.
                    time.sleep(self._nav2_settle_seconds)
                    print("✅ Nav2 behavior servers should be ready.")
                    return
            time.sleep(NAV2_ACTIVATION_POLL_SECONDS)

        raise PreciseMotionError(
            f"Nav2 did not become active within "
            f"{NAV2_ACTIVATION_TIMEOUT_SECONDS}s after calling "
            f"/robot/start_navigation. Cannot proceed with Nav2 engine."
        )

    def _wait_for_precise_move(self, timeout: float = 30.0) -> tuple:
        """Wait for robot_manager.py to start and then finish precise motion.

        Phase 1: Wait up to PRECISE_MOVE_START_TIMEOUT_SECONDS for the
        ``/robot/precise_move_active`` topic to go ``True`` (started).
        Phase 2: Wait up to *timeout* seconds for it to go ``False``
        (finished).

        This mirrors the exact logic used in robot_agent.py.

        Args:
            timeout: Maximum seconds to wait for the motion to complete
                     (Phase 2 only).

        Returns:
            tuple[bool, bool, float]: (started, finished, active_duration_seconds).
        """
        started = False
        finished = False
        active_duration = 0.0
        phase1_end_time = 0.0

        # Phase 1: wait for the manager to start (precise_active → True)
        start_deadline = time.time() + PRECISE_MOVE_START_TIMEOUT_SECONDS
        while time.time() < start_deadline:
            if self._move_cancel.is_set():
                return started, finished, active_duration
            with self._precise_active_lock:
                if self._precise_active:
                    started = True
                    phase1_end_time = time.time()
                    break
            time.sleep(PRECISE_MOVE_POLL_INTERVAL_SECONDS)

        if not started:
            print("⚠️ Precise move command timed out before robot_manager started.")
            return started, finished, active_duration

        # Phase 2: wait for move to complete (precise_active → False)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._move_cancel.is_set():
                return started, finished, active_duration
            with self._precise_active_lock:
                if not self._precise_active:
                    finished = True
                    active_duration = time.time() - phase1_end_time
                    return started, finished, active_duration
            time.sleep(PRECISE_MOVE_POLL_INTERVAL_SECONDS)

        print(f"⚠️ Precise move command timed out after {timeout:.1f}s while moving.")
        return started, finished, active_duration

    # ── Public drive methods ────────────────────────────────────────────

    def drive_distance(
        self,
        dist,
        speed=DEFAULT_DRIVE_SPEED,
        engine=None,
        timeout=DRIVE_DISTANCE_TIMEOUT_SECONDS,
    ):
        """Drive the robot a specific distance. Blocks until native engine completes.

        Args:
            dist (float): Distance in meters (+forward, -backward).
            speed (float): Linear speed in m/s.
            engine (str|PreciseMotionEngine): internal (odom) or nav2 (costmap aware).
            timeout (float): Max allowed seconds.

        Returns:
            bool: True if completed (or safely stopped by Nav2).
        """
        # ── Guard: fail-fast distance check (before any setup) ──
        if abs(dist) > MAX_PRECISE_DISTANCE:
            raise PreciseMotionError(
                f"drive_distance: requested {abs(dist):.3f}m exceeds "
                f"MAX_PRECISE_DISTANCE ({MAX_PRECISE_DISTANCE}m)"
            )

        # ── Intercept negative distance to protect rear blindspot ──
        if dist < 0:
            print(f"🔄 Negative distance ({dist:.3f}m) requested. Rotating 180° to drive forward, then rotating back to preserve lidar coverage.")
            # 1. Turn 180 degrees
            if not self.rotate_angle(180.0, engine=engine, timeout=timeout):
                return False
            # 2. Drive the absolute distance forward
            if not self.drive_distance(abs(dist), speed, engine=engine, timeout=timeout):
                return False
            # 3. Turn -180 degrees back to original orientation
            return self.rotate_angle(-180.0, engine=engine, timeout=timeout)

        resolved = self._resolve_engine(engine)
        use_nav2 = resolved == PreciseMotionEngine.NAV2

        # ── Pre-flight: ensure Nav2 is up if needed ──
        if use_nav2:
            self._ensure_nav2_active()

        self._move_cancel.clear()
        self._precise_moving_flag.set()

        try:
            payload = {
                "mode": "move",
                "distance": float(dist),
                "speed": float(abs(speed)),
                "use_nav2": use_nav2,
            }
            print(
                f"▶️ drive_distance: {dist:.3f}m @ {abs(speed):.2f} m/s "
                f"(engine={resolved.value})"
            )
            self._publish_precise_command(payload)

            # Calculate a sensible timeout: travel time + generous buffer
            buffer = 20.0
            effective_timeout = max(timeout, abs(dist) / abs(speed) + buffer)

            started, finished, active_duration = self._wait_for_precise_move(effective_timeout)

            if started and finished:
                expected_time = abs(dist) / abs(speed)
                if active_duration < expected_time * 0.5:
                    print(f"⚠️ drive_distance completed suspiciously fast ({active_duration:.2f}s < {expected_time * 0.5:.2f}s).")
                    raise PreciseMotionPrematureCompletionError(
                        f"drive_distance completed in {active_duration:.2f}s, which is suspiciously fast compared "
                        f"to the expected minimum time of {expected_time:.2f}s. The behavior server may not be ready."
                    )
                
                print(f"✅ drive_distance: completed {dist:.3f}m")
                return True
            elif not started:
                print(f"❌ drive_distance: robot_manager did not start the command")
                return False
            else:
                print(f"❌ drive_distance: command did not complete within timeout")
                return False

        except Exception as exc:
            print(f"❌ drive_distance error: {exc}")
            traceback.print_exc()
            return False
        finally:
            self._precise_moving_flag.clear()

    def rotate_angle(
        self,
        angle,
        speed=DEFAULT_ROTATE_SPEED,
        engine=None,
        timeout=ROTATE_ANGLE_TIMEOUT_SECONDS,
    ):
        """Rotate the robot by a specific angle. Blocks until native engine completes.

        Args:
            angle (float): Rotation angle in degrees (+CCW, -CW).
            speed (float): Rotational speed in deg/s.
            engine (str|PreciseMotionEngine): internal (odom) or nav2 (costmap aware).
            timeout (float): Max allowed seconds.

        Returns:
            bool: True if completed (or safely stopped by Nav2).
        """
        resolved = self._resolve_engine(engine)
        use_nav2 = resolved == PreciseMotionEngine.NAV2

        # ── Pre-flight: ensure Nav2 is up if needed ──
        if use_nav2:
            self._ensure_nav2_active()

        self._move_cancel.clear()
        self._precise_moving_flag.set()

        try:
            payload = {
                "mode": "rotate",
                "angle": float(angle),
                "speed": float(abs(speed)),
                "use_nav2": use_nav2,
            }
            print(
                f"▶️ rotate_angle: {angle:.1f}° @ {abs(speed):.1f} deg/s "
                f"(engine={resolved.value})"
            )
            self._publish_precise_command(payload)

            # Calculate a sensible timeout: rotation time + generous buffer
            buffer = 20.0
            effective_timeout = max(timeout, abs(angle) / abs(speed) + buffer)

            started, finished, active_duration = self._wait_for_precise_move(effective_timeout)

            if started and finished:
                expected_time = abs(angle) / abs(speed)
                if active_duration < expected_time * 0.5:
                    print(f"⚠️ rotate_angle completed suspiciously fast ({active_duration:.2f}s < {expected_time * 0.5:.2f}s).")
                    raise PreciseMotionPrematureCompletionError(
                        f"rotate_angle completed in {active_duration:.2f}s, which is suspiciously fast compared "
                        f"to the expected minimum time of {expected_time:.2f}s. The behavior server may not be ready."
                    )
                
                print(f"✅ rotate_angle: completed {angle:.1f}°")
                return True
            elif not started:
                print(f"❌ rotate_angle: robot_manager did not start the command")
                return False
            else:
                print(f"❌ rotate_angle: command did not complete within timeout")
                return False

        except Exception as exc:
            print(f"❌ rotate_angle error: {exc}")
            traceback.print_exc()
            return False
        finally:
            self._precise_moving_flag.clear()

    def drive_and_rotate(
        self,
        dist,
        angle,
        speed=DEFAULT_DRIVE_SPEED,
        turn_speed=DEFAULT_ROTATE_SPEED,
        engine=None,
        timeout=DRIVE_DISTANCE_TIMEOUT_SECONDS,
    ):
        """Drive a distance then rotate by an angle. Stops if either step fails.

        Args:
            dist (float): Distance in meters (+forward, -backward).
            angle (float): Rotation angle in degrees (+CCW, -CW).
            speed (float): Linear speed in m/s.
            turn_speed (float): Rotational speed in deg/s.
            engine (str|PreciseMotionEngine): Execution engine.
            timeout (float): Timeout per step in seconds.

        Returns:
            bool: True if both steps succeeded.
        """
        if not self.drive_distance(dist, speed, engine=engine, timeout=timeout):
            return False
        return self.rotate_angle(angle, turn_speed, engine=engine, timeout=timeout)

    # ── Teardown ─────────────────────────────────────────────────────

    def _queue_shutdown(self):
        """Clean up queue and precise motion bridge resources during controller shutdown."""
        self.clear_queue()
        # Give the worker thread a moment to exit
        if self._queue_thread and self._queue_thread.is_alive():
            self._queue_thread.join(timeout=2.0)

        safe_unsubscribe(getattr(self, '_precise_active_sub', None))
        safe_unsubscribe(getattr(self, '_nav_active_sub', None))
        safe_unadvertise(getattr(self, '_precise_move_pub', None))
