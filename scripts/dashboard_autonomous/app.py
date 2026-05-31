#!/usr/bin/env python3
"""
Standalone Flask + SocketIO server for BonicBot Autonomous Exploration Dashboard.
Runs on port 5001 to avoid conflicts with the main dashboard.

All long-running operations (setup_for_exploration, wait_for_map_complete) run in
background threads and stream progress to the frontend via SocketIO events.
"""
import os
import sys
import time
import math
import atexit
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO

# Add the bonicbot-bridge root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from bonicbot_bridge.core import BonicBot
from bonicbot_bridge.autonomous import ExploreError, ExploreTimeoutError

app = Flask(__name__)
# Bug 10 fix: Increase max payload size to 50MB to allow large SLAM maps to transmit without dropping the websocket
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*', max_http_buffer_size=50_000_000)

# ── Global State ─────────────────────────────────────────────────────────
# Tracks which background operation is running so we don't double-start.
_setup_running = False
_wait_running = False

print("⏳ Connecting to ROS bridge for Autonomous Dashboard...")
try:
    bot = BonicBot()
    bot.connect()
    system  = bot.system
    motion  = bot.motion
    explore = bot.explore

    if not (bot.ros and bot.ros.is_connected):
        raise RuntimeError("bot.connect() succeeded but ROS bridge is not connected")

    print("✅ Connected successfully.")
except Exception as e:
    print(f"⚠️ WARNING: Could not connect to ROS bridge. {e}")
    bot, system, motion, explore = None, None, None, None

# Register explore_lite lifecycle callback to forward events to frontend
if explore:
    def _on_lifecycle(status):
        socketio.emit('explore_lifecycle', {'status': status}, namespace='/')
    explore.set_lifecycle_callback(_on_lifecycle)

def _shutdown():
    print("🛑 Shutting down Autonomous Dashboard...")
    if bot:
        bot.disconnect()

atexit.register(_shutdown)

# ── Callbacks ────────────────────────────────────────────────────────────

def _on_map(msg: dict) -> None:
    try:
        info = msg.get('info', {})
        socketio.emit('map_update', {
            'ts': int(time.time() * 1000),
            'resolution': info.get('resolution', 0.05),
            'width': info.get('width', 0),
            'height': info.get('height', 0),
            'data': msg.get('data', []),
            'origin': info.get('origin', {}),
        }, namespace='/')
    except Exception as exc:
        print(f"Error in _on_map: {exc}")

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def _on_odom(msg):
    try:
        pose = msg.get('pose', {}).get('pose', {})
        position = pose.get('position', {})
        orientation = pose.get('orientation', {})
        yaw = euler_from_quaternion(
            orientation.get('x', 0),
            orientation.get('y', 0),
            orientation.get('z', 0),
            orientation.get('w', 1.0)
        )
        socketio.emit('robot_pose', {
            'ts': int(time.time() * 1000),
            'x': position.get('x', 0.0),
            'y': position.get('y', 0.0),
            'yaw': yaw
        }, namespace='/')
    except Exception as e:
        print(f"Error in _on_odom: {e}")

# ── Subscriptions ────────────────────────────────────────────────────────
# Use SDK subscribe methods — this is what the dashboard is testing.

if system:
    system.subscribe_to_map(_on_map, throttle_rate=500)
    system.subscribe_to_odom(_on_odom, throttle_rate=100)

# ── Background State Poller ──────────────────────────────────────────────

def state_poller():
    """Polls exploration state and emits to frontend every second."""
    while True:
        if explore:
            diag = explore.diagnostics()
            socketio.emit('explore_status', diag, namespace='/')
        time.sleep(1.0)

socketio.start_background_task(state_poller)

# ── Routes ───────────────────────────────────────────────────────────────

@socketio.on('connect')
def handle_connect():
    if explore:
        socketio.emit('explore_status', explore.diagnostics(), namespace='/')

@app.route('/')
def index():
    # Bug 3 fix: use bot.ros.is_connected (property), not bot.is_connected()
    is_conn = bot.ros.is_connected if bot and bot.ros else False
    return render_template('index.html', connected=is_conn)

# ── Exploration Setup (Background) ───────────────────────────────────────

@app.route('/api/explore/setup', methods=['POST'])
def explore_setup():
    global _setup_running
    if not explore:
        return jsonify({'success': False, 'message': 'Not connected to ROS'}), 500
    if _setup_running:
        return jsonify({'success': False, 'message': 'Setup already in progress'}), 409

    # Bug 4 fix: set flag BEFORE starting background task to prevent race
    _setup_running = True

    def _run_setup():
        global _setup_running

        def _on_progress(step, total, stage, message):
            pct = int((step / total) * 100)
            socketio.emit('setup_progress', {
                'step': step,
                'total': total,
                'stage': stage,
                'message': message,
                'percent': pct,
            }, namespace='/')

        try:
            explore.setup_for_exploration(progress_callback=_on_progress)
            socketio.emit('setup_result', {'success': True, 'message': 'Exploration started!'}, namespace='/')
        except Exception as e:
            socketio.emit('setup_result', {'success': False, 'message': str(e)}, namespace='/')
        finally:
            _setup_running = False

    socketio.start_background_task(_run_setup)
    return jsonify({'success': True, 'message': 'Setup started in background'}), 202

# ── Wait for Map Complete (Background) ───────────────────────────────────

@app.route('/api/explore/wait', methods=['POST'])
def explore_wait():
    global _wait_running
    if not explore:
        return jsonify({'success': False, 'message': 'Not connected to ROS'}), 500
    if _wait_running:
        return jsonify({'success': False, 'message': 'Wait already in progress'}), 409

    # Bug 4 fix: set flag BEFORE starting background task to prevent race
    _wait_running = True

    data = request.json or {}
    min_area = float(data.get('min_area', 10.0))
    timeout = float(data.get('timeout', 300.0))

    def _run_wait():
        global _wait_running

        def _on_map_progress(current_area, target_area, elapsed, exhausted):
            pct = min(100, int((current_area / target_area) * 100)) if target_area > 0 else 0
            socketio.emit('map_progress', {
                'current_area': round(current_area, 2),
                'target_area': round(target_area, 2),
                'elapsed': round(elapsed, 1),
                'percent': pct,
                'frontiers_exhausted': exhausted,
            }, namespace='/')

        try:
            explore.wait_for_map_complete(min_area, timeout=timeout, progress_callback=_on_map_progress)
            socketio.emit('map_result', {'success': True, 'message': 'Map complete!'}, namespace='/')
        except ExploreTimeoutError as e:
            socketio.emit('map_result', {'success': False, 'message': str(e)}, namespace='/')
        except Exception as e:
            socketio.emit('map_result', {'success': False, 'message': str(e)}, namespace='/')
        finally:
            _wait_running = False

    socketio.start_background_task(_run_wait)
    return jsonify({'success': True, 'message': f'Waiting for {min_area}m² (timeout {timeout}s)'}), 202

# ── Stop Exploration ─────────────────────────────────────────────────────

@app.route('/api/explore/stop', methods=['POST'])
def explore_stop():
    if not explore:
        return jsonify({'success': False, 'message': 'Not connected to ROS'}), 500
    try:
        explore.stop_explore()
        return jsonify({'success': True, 'message': 'Exploration stopped.'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

# ── Start / Resume Exploration ───────────────────────────────────────────
# Bug 5 fix: missing endpoint — allows resuming exploration without full setup.

@app.route('/api/explore/start', methods=['POST'])
def explore_start():
    if not explore:
        return jsonify({'success': False, 'message': 'Not connected'}), 500
    try:
        explore.start_explore()
        return jsonify({'success': True, 'message': 'Exploration resumed'})
    except ExploreError as e:
        return jsonify({'success': False, 'message': str(e)}), 500

if __name__ == '__main__':
    print("🚀 Starting Autonomous Dashboard on http://0.0.0.0:5001")
    socketio.run(app, host='0.0.0.0', port=5001, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
