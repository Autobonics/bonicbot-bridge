"""
Thread-safe command queue mixin for MotionController.

Provides enqueue_move(), run_queue(), clear_queue(), and draw_square()
as a mixin class that MotionController inherits from.  All queue state
is initialised lazily by _init_queue() which must be called once from
the host class's __init__.

Architecture notes
──────────────────
• roslibpy callbacks fire on their own background threads; there is
  no rclpy executor in this codebase.
• A single daemon thread (_queue_worker) pulls commands from a
  stdlib queue.Queue and dispatches them to drive_distance() /
  rotate_angle() which already do their own closed-loop control.
• clear_queue() is safe to call from any thread (including roslibpy
  callbacks) — it touches only a threading.Event and queue.Queue
  which are both inherently thread-safe.
"""

import enum
import math
import queue
import threading
import time
import traceback

from roslibpy import ActionClient, Goal, Topic

from .exceptions import NavigationError, PreciseMotionError
from .utils import DIFF_CONT_ODOM_TOPIC, MAX_PRECISE_DISTANCE, ODOMETRY_MESSAGE_TYPE, safe_unsubscribe

# ── Queue command types ────────────────────────────────────────────────
CMD_TYPE_DRIVE = "drive"
CMD_TYPE_ROTATE = "rotate"

# ── Defaults ───────────────────────────────────────────────────────────
DEFAULT_QUEUE_DRIVE_SPEED = 0.3      # m/s
DEFAULT_QUEUE_ROTATE_SPEED = 45.0    # deg/s
DEFAULT_QUEUE_TIMEOUT = 30.0         # seconds per command
DEFAULT_QUEUE_ENGINE = "internal"
QUEUE_POLL_INTERVAL_SECONDS = 0.05   # 50 ms between empty-queue checks

# ═══════════════════════════════════════════════════════════════════════════════
# NEW — Precise-motion constants (drive_distance / rotate_angle / drive_and_rotate)
# ═══════════════════════════════════════════════════════════════════════════════

# Odometry topic (shared with sensors.py but independent subscription)

# Internal engine control loop
ODOM_POLL_INTERVAL_SECONDS = 0.05  # 20 Hz control loop
DRIVE_DISTANCE_TIMEOUT_SECONDS = 30.0
ROTATE_ANGLE_TIMEOUT_SECONDS = 30.0
DISTANCE_TOLERANCE_METERS = 0.02  # 2 cm
ANGLE_TOLERANCE_DEGREES = 1.0  # 1°
ODOM_WAIT_TIMEOUT_SECONDS = 5.0

# Nav2 action servers
DRIVE_ON_HEADING_ACTION = "/drive_on_heading"
SPIN_ACTION = "/spin"
DRIVE_ON_HEADING_ACTION_TYPE = "nav2_msgs/action/DriveOnHeading"
SPIN_ACTION_TYPE = "nav2_msgs/action/Spin"
ACTION_SERVER_TIMEOUT_SECONDS = 10.0

# Engine enum — use this for type-safe engine selection across the codebase
class PreciseMotionEngine(enum.Enum):
    """Execution engine for precise motion commands (drive_distance / rotate_angle).

    Members:
        INTERNAL: Closed-loop control via /cmd_vel + odometry feedback.
        NAV2: Delegates to Nav2 action servers (DriveOnHeading / Spin).
    """
    INTERNAL = "internal"
    NAV2 = "nav2"

# Deprecated string aliases — kept for backward compatibility
ENGINE_INTERNAL = PreciseMotionEngine.INTERNAL.value
ENGINE_NAV2 = PreciseMotionEngine.NAV2.value

# Default speeds for drive methods
DEFAULT_DRIVE_SPEED = 0.3  # m/s
DEFAULT_ROTATE_SPEED = 45.0  # deg/s

# ═══════════════════════════════════════════════════════════════════════════════


class QueueMixin:
    """Thread-safe command queue for sequential motion execution.

    Mix this into MotionController to gain enqueue_move(), run_queue(),
    clear_queue() and draw_square().  The host class MUST call
    ``self._init_queue()`` at the end of its ``__init__`` and
    ``self._queue_shutdown()`` inside ``shutdown()``.
    """

    # ── Initialisation (called from MotionController.__init__) ───────

    def _init_queue(self):
        """Initialise queue and precise motion infrastructure.  Call once from __init__."""
        self._cmd_queue = queue.Queue()
        self._queue_cancel = threading.Event()
        self._queue_done_event = threading.Event()
        self._queue_thread = None
        self._queue_lock = threading.Lock()
        # Holds the boolean result of the most recent run_queue() execution
        self._queue_result = True

        # NEW — Odometry subscription for closed-loop drive methods
        self._odom_sub = Topic(self.ros, DIFF_CONT_ODOM_TOPIC, ODOMETRY_MESSAGE_TYPE)
        self._current_odom = None
        self._odom_sub.subscribe(self._odom_callback)

        # Precise-motion infrastructure
        self._default_engine = PreciseMotionEngine.INTERNAL
        self._precise_moving_flag = threading.Event()  # SET while executing

    # ── Public API ───────────────────────────────────────────────────

    def enqueue_move(self, cmd_list):
        """Push motion commands onto the queue (non-blocking).

        Each element of *cmd_list* is a dict with the following keys:

        ┌──────────┬──────────────────────────────────────────────────┐
        │ Key      │ Description                                      │
        ├──────────┼──────────────────────────────────────────────────┤
        │ type     │ ``'drive'`` or ``'rotate'`` (required)           │
        │ value    │ Distance in metres / angle in degrees (required) │
        │ speed    │ m/s or deg/s  (optional, uses defaults)          │
        │ engine   │ ``'internal'`` or ``'nav2'`` (optional)          │
        │ timeout  │ Per-command timeout in seconds (optional)        │
        └──────────┴──────────────────────────────────────────────────┘

        Args:
            cmd_list (list[dict]): List of command dictionaries.

        Returns:
            None

        Thread-safety:
            queue.Queue.put() is inherently thread-safe.  This method
            can be called from any thread, including roslibpy callbacks.
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
        """Start executing the queued commands on a background thread.

        If a background execution is already running, new commands that
        were enqueued will simply be picked up by the existing thread.

        Args:
            block (bool): If ``True`` (default), block the caller until
                every queued command has finished (or a failure occurs).
                If ``False``, return immediately after starting the
                background thread.

        Returns:
            bool: ``True`` if all commands succeeded.  ``False`` if any
                  command failed, timed out, or was cancelled.  When
                  *block* is ``False`` the return value is always
                  ``True`` (the real result is not yet known).

        Thread-safety:
            _queue_lock serialises thread creation.  Only one worker
            thread can exist at a time.
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

        Safe to call from any thread (including roslibpy callbacks).

        Behaviour:
            1. Drain all pending items from the queue.
            2. Signal the background worker to stop via _queue_cancel.
            3. Signal the currently running motion primitive to stop
               via _move_cancel (inherited from MotionController).
            4. Publish a zero-velocity Twist so the robot halts.

        Returns:
            None
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

        Enqueues 4× [drive *side_m*, rotate 90°] and executes them
        sequentially.  Blocks until the full square is complete or a
        failure occurs.

        Args:
            side_m (float): Length of each side in metres.
            speed (float): Linear speed in m/s (default 0.3).
            turn_speed (float): Rotational speed in deg/s (default 45).
            engine (str): ``'internal'`` or ``'nav2'`` (default
                ``'internal'``).
            timeout (float): Per-command timeout in seconds (default 30).

        Returns:
            bool: ``True`` if the full square was completed, ``False``
                  if any leg or turn failed.
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
        """Daemon thread target — pull and execute commands one by one.

        Error handling policy:
        • If any command returns False → clear_queue(), set result to
          False, signal _queue_done_event, exit.
        • On unexpected exception → same as above, plus traceback.
        • finally block always sends zero Twist.
        """
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

    # ── Teardown ─────────────────────────────────────────────────────

    # ═══════════════════════════════════════════════════════════════════════
    # NEW — Precise-motion methods (drive_distance / rotate_angle / drive_and_rotate)
    #
    # Helpers: _odom_callback, _get_position_from_odom, _get_yaw_from_odom,
    #          _normalize_angle, _wait_for_odom, _send_action_goal
    # Public:  set_default_engine, is_precise_moving, drive_distance, rotate_angle, drive_and_rotate
    # ═══════════════════════════════════════════════════════════════════════

    def set_default_engine(self, engine):
        """Set the default execution engine for precise motion commands.

        The default engine is used when ``drive_distance()``,
        ``rotate_angle()``, or ``drive_and_rotate()`` are called without
        an explicit ``engine`` argument.

        Args:
            engine (PreciseMotionEngine): The engine to use by default.
                Must be a ``PreciseMotionEngine`` enum member.

        Raises:
            TypeError: If *engine* is not a ``PreciseMotionEngine`` member.

        Returns:
            None
        """
        if not isinstance(engine, PreciseMotionEngine):
            raise TypeError(
                f"engine must be a PreciseMotionEngine member, "
                f"got {type(engine).__name__}"
            )
        self._default_engine = engine

    def is_precise_moving(self):
        """Check if a precise motion command is currently executing.

        Returns ``True`` while any ``drive_distance`` / ``rotate_angle`` /
        ``drive_and_rotate`` command is actively running on either engine.
        The flag is set before the first velocity publish or action-goal
        send and cleared in a ``finally`` block, so it resets even if an
        exception is raised mid-motion.

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
            ValueError: If *engine* is an unrecognised string.
            TypeError: If *engine* is not str, PreciseMotionEngine, or None.
        """
        if engine is None:
            return self._default_engine
        if isinstance(engine, PreciseMotionEngine):
            return engine
        if isinstance(engine, str):
            try:
                return PreciseMotionEngine(engine)
            except ValueError:
                raise ValueError(
                    f"Unknown engine '{engine}'. "
                    f"Valid engines: {[e.value for e in PreciseMotionEngine]}"
                )
        raise TypeError(
            f"engine must be PreciseMotionEngine, str, or None — "
            f"got {type(engine).__name__}"
        )

    # ── Odometry helpers ────────────────────────────────────────────────

    def _odom_callback(self, msg):
        """Store latest raw odometry message for closed-loop control."""
        self._current_odom = msg

    def _get_position_from_odom(self):
        """
        Extract (x, y) position from current odometry.

        Returns:
            tuple[float, float] | None: (x, y) in meters, or None if no data.
        """
        if self._current_odom is None:
            return None
        pos = self._current_odom["pose"]["pose"]["position"]
        return (pos["x"], pos["y"])

    def _get_yaw_from_odom(self):
        """
        Extract yaw angle from current odometry quaternion.

        Returns:
            float | None: Yaw in degrees [-180, 180], or None if no data.
        """
        if self._current_odom is None:
            return None
        ori = self._current_odom["pose"]["pose"]["orientation"]
        yaw_rad = 2.0 * math.atan2(ori["z"], ori["w"])
        return math.degrees(yaw_rad)

    @staticmethod
    def _normalize_angle(angle_deg):
        """
        Normalize an angle delta to the range [-180, 180] degrees.

        Args:
            angle_deg (float): Angle in degrees.

        Returns:
            float: Normalized angle in degrees.
        """
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def _wait_for_odom(self, timeout=ODOM_WAIT_TIMEOUT_SECONDS):
        """
        Block until the first odometry message is received.

        Args:
            timeout (float): Maximum seconds to wait.

        Returns:
            bool: True if odometry data is available, False on timeout.
        """
        start = time.time()
        while self._current_odom is None and (time.time() - start) < timeout:
            time.sleep(ODOM_POLL_INTERVAL_SECONDS)
        return self._current_odom is not None

    # ── Nav2 action helper ──────────────────────────────────────────────

    def _send_action_goal(self, server_name, action_type, goal_msg, timeout):
        """
        Send a goal to a Nav2 action server and wait for the result.

        Args:
            server_name (str): Action server topic (e.g. '/drive_on_heading').
            action_type (str): Action type string (e.g. 'nav2_msgs/action/DriveOnHeading').
            goal_msg (dict): Goal message dictionary.
            timeout (float): Maximum seconds to wait for the result.

        Returns:
            bool: True if the action succeeded, False otherwise.
        """
        result_event = threading.Event()
        result_holder = {"success": False}

        def on_result(result):
            # Nav2 action result — status 3 == SUCCEEDED (from action_msgs/GoalStatus)
            status = result.get("status", -1)
            result_holder["success"] = status == 3
            result_event.set()

        try:
            client = ActionClient(self.ros, server_name, action_type)
            goal = Goal(client, goal_msg)
            goal.on("result", on_result)
            goal.send()

            completed = result_event.wait(timeout=timeout)

            if not completed:
                print(f"⏰ Action '{server_name}' timed out after {timeout}s")
                try:
                    goal.cancel()
                except Exception:
                    pass
                return False

            return result_holder["success"]

        except Exception as exc:
            print(f"❌ Action '{server_name}' failed: {exc}")
            traceback.print_exc()
            return False

    # ── Public drive methods ────────────────────────────────────────────

    def drive_distance(
        self,
        dist,
        speed=DEFAULT_DRIVE_SPEED,
        engine=None,
        timeout=DRIVE_DISTANCE_TIMEOUT_SECONDS,
    ):
        """
        Drive the robot a specific distance using odometry feedback or Nav2.

        Args:
            dist (float): Distance to drive in meters.
                          Positive = forward, negative = backward.
            speed (float): Linear speed in m/s (always positive; direction is
                          determined by the sign of *dist*).
            engine (PreciseMotionEngine | str | None): Execution engine.
                ``PreciseMotionEngine.INTERNAL`` — cmd_vel + odom,
                ``PreciseMotionEngine.NAV2`` — DriveOnHeading action.
                ``None`` uses the default engine (see ``set_default_engine``).
                Plain strings ``'internal'`` / ``'nav2'`` are accepted for
                backward compatibility.
            timeout (float): Maximum seconds allowed for the manoeuvre.

        Returns:
            bool: True if the target distance was reached.

        Raises:
            PreciseMotionError: If ``abs(dist)`` exceeds
                ``MAX_PRECISE_DISTANCE`` (fail-fast, no side effects).
            NavigationError: On odometry timeout or Nav2 non-success.
        """
        # ── Guard: fail-fast distance check (before any setup) ──
        if abs(dist) > MAX_PRECISE_DISTANCE:
            raise PreciseMotionError(
                f"drive_distance: requested {abs(dist):.3f}m exceeds "
                f"MAX_PRECISE_DISTANCE ({MAX_PRECISE_DISTANCE}m)"
            )

        resolved = self._resolve_engine(engine)
        if resolved == PreciseMotionEngine.NAV2:
            return self._drive_distance_nav2(dist, speed, timeout)
        return self._drive_distance_internal(dist, speed, timeout)

    def _drive_distance_internal(self, dist, speed, timeout):
        """Internal engine: closed-loop drive using /cmd_vel + odometry."""
        if not self._wait_for_odom():
            print("⚠️ drive_distance: no odometry data — aborting")
            return False

        abs_dist = abs(dist)
        direction = 1.0 if dist >= 0 else -1.0
        cmd_speed = abs(speed) * direction

        start_pos = self._get_position_from_odom()
        if start_pos is None:
            return False

        self._move_cancel.clear()
        start_time = time.time()
        self._precise_moving_flag.set()

        try:
            while not self._move_cancel.is_set():
                elapsed = time.time() - start_time
                if elapsed >= timeout:
                    raise NavigationError(
                        f"_drive_distance_internal: odometry timeout — "
                        f"elapsed {elapsed:.1f}s >= timeout {timeout:.1f}s"
                    )

                cur_pos = self._get_position_from_odom()
                if cur_pos is None:
                    time.sleep(ODOM_POLL_INTERVAL_SECONDS)
                    continue

                dx = cur_pos[0] - start_pos[0]
                dy = cur_pos[1] - start_pos[1]
                traveled = math.sqrt(dx * dx + dy * dy)

                if traveled >= abs_dist - DISTANCE_TOLERANCE_METERS:
                    self.stop()
                    print(
                        f"✅ drive_distance: reached {traveled:.3f}m "
                        f"(target {abs_dist:.3f}m)"
                    )
                    return True

                # Publish velocity command
                self.move(linear_x=cmd_speed)
                time.sleep(ODOM_POLL_INTERVAL_SECONDS)

        except NavigationError:
            raise
        except Exception as exc:
            print(f"❌ drive_distance internal error: {exc}")
            traceback.print_exc()
        finally:
            self._precise_moving_flag.clear()
            self.stop()

        return False

    def _drive_distance_nav2(self, dist, speed, timeout):
        """Nav2 engine: DriveOnHeading action client."""
        self._precise_moving_flag.set()
        try:
            goal_msg = {
                "target": {"x": float(dist), "y": 0.0, "z": 0.0},
                "speed": float(abs(speed)),
                "time_allowance": {"sec": int(timeout), "nanosec": 0},
            }
            success = self._send_action_goal(
                DRIVE_ON_HEADING_ACTION, DRIVE_ON_HEADING_ACTION_TYPE,
                goal_msg, timeout,
            )
            if success:
                print(f"✅ drive_distance (nav2): completed {dist:.3f}m")
                return True
            # OBSTACLE_STOP: Nav2 BT has exhausted retries — raise immediately
            raise NavigationError(
                f"drive_distance (nav2): Nav2 returned non-success for "
                f"{dist:.3f}m — obstacle stop or BT failure"
            )
        finally:
            self._precise_moving_flag.clear()

    def rotate_angle(
        self,
        angle,
        speed=DEFAULT_ROTATE_SPEED,
        engine=None,
        timeout=ROTATE_ANGLE_TIMEOUT_SECONDS,
    ):
        """
        Rotate the robot by a specific angle using odometry feedback or Nav2.

        Args:
            angle (float): Rotation angle in degrees.
                          Positive = counter-clockwise (CCW),
                          negative = clockwise (CW).
            speed (float): Rotational speed in deg/s (always positive;
                          direction is determined by the sign of *angle*).
            engine (PreciseMotionEngine | str | None): Execution engine.
                ``None`` uses the default engine (see ``set_default_engine``).
            timeout (float): Maximum seconds allowed for the manoeuvre.

        Returns:
            bool: True if the target angle was reached.

        Raises:
            NavigationError: On odometry timeout or Nav2 non-success.
        """
        resolved = self._resolve_engine(engine)
        if resolved == PreciseMotionEngine.NAV2:
            return self._rotate_angle_nav2(angle, speed, timeout)
        return self._rotate_angle_internal(angle, speed, timeout)

    def _rotate_angle_internal(self, angle, speed, timeout):
        """Internal engine: closed-loop rotation using /cmd_vel + odometry yaw."""
        if not self._wait_for_odom():
            print("⚠️ rotate_angle: no odometry data — aborting")
            return False

        abs_angle = abs(angle)
        # Positive angle = CCW = positive angular_z
        direction = 1.0 if angle >= 0 else -1.0
        cmd_speed = abs(speed) * direction  # deg/s, move() converts to rad/s

        start_yaw = self._get_yaw_from_odom()
        if start_yaw is None:
            return False

        accumulated = 0.0
        prev_yaw = start_yaw
        self._move_cancel.clear()
        start_time = time.time()
        self._precise_moving_flag.set()

        try:
            while not self._move_cancel.is_set():
                elapsed = time.time() - start_time
                if elapsed >= timeout:
                    raise NavigationError(
                        f"_rotate_angle_internal: odometry timeout — "
                        f"elapsed {elapsed:.1f}s >= timeout {timeout:.1f}s"
                    )

                cur_yaw = self._get_yaw_from_odom()
                if cur_yaw is None:
                    time.sleep(ODOM_POLL_INTERVAL_SECONDS)
                    continue

                delta = self._normalize_angle(cur_yaw - prev_yaw)
                accumulated += delta
                prev_yaw = cur_yaw

                if abs(accumulated) >= abs_angle - ANGLE_TOLERANCE_DEGREES:
                    self.stop()
                    print(
                        f"✅ rotate_angle: rotated {accumulated:.1f}° "
                        f"(target {angle:.1f}°)"
                    )
                    return True

                # Publish angular velocity (deg/s — move() converts to rad/s)
                self.move(angular_z=cmd_speed)
                time.sleep(ODOM_POLL_INTERVAL_SECONDS)

        except NavigationError:
            raise
        except Exception as exc:
            print(f"❌ rotate_angle internal error: {exc}")
            traceback.print_exc()
        finally:
            self._precise_moving_flag.clear()
            self.stop()

        return False

    def _rotate_angle_nav2(self, angle, speed, timeout):
        """Nav2 engine: Spin action client."""
        self._precise_moving_flag.set()
        try:
            target_yaw_rad = math.radians(angle)
            goal_msg = {
                "target_yaw": float(target_yaw_rad),
                "time_allowance": {"sec": int(timeout), "nanosec": 0},
            }
            success = self._send_action_goal(
                SPIN_ACTION, SPIN_ACTION_TYPE, goal_msg, timeout
            )
            if success:
                print(f"✅ rotate_angle (nav2): completed {angle:.1f}°")
                return True
            # OBSTACLE_STOP: Nav2 BT has exhausted retries — raise immediately
            raise NavigationError(
                f"rotate_angle (nav2): Nav2 returned non-success for "
                f"{angle:.1f}° — obstacle stop or BT failure"
            )
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
        """
        Drive a distance then rotate by an angle. Stops if either step fails.

        Args:
            dist (float): Distance in meters (positive = forward,
                         negative = backward).
            angle (float): Rotation angle in degrees (positive = CCW,
                          negative = CW).
            speed (float): Linear speed in m/s for the drive phase.
            turn_speed (float): Rotational speed in deg/s for the rotation phase.
            engine (PreciseMotionEngine | str | None): Execution engine for
                both sub-steps.  ``None`` uses the default engine.
            timeout (float): Timeout per step in seconds.

        Returns:
            bool: True if both steps succeeded, False if either failed.

        Raises:
            PreciseMotionError: If ``abs(dist)`` exceeds MAX_PRECISE_DISTANCE.
            NavigationError: On odometry timeout or Nav2 non-success.
        """
        if not self.drive_distance(dist, speed, engine=engine, timeout=timeout):
            return False
        return self.rotate_angle(angle, turn_speed, engine=engine, timeout=timeout)

    # ── Teardown ─────────────────────────────────────────────────────

    def _queue_shutdown(self):
        """Clean up queue and precise motion resources during controller shutdown."""
        self.clear_queue()
        # Give the worker thread a moment to exit
        if self._queue_thread and self._queue_thread.is_alive():
            self._queue_thread.join(timeout=2.0)
        
        safe_unsubscribe(getattr(self, '_odom_sub', None))

