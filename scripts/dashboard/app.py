#!/usr/bin/env python3
"""
Flask + SocketIO server for BonicBot Dashboard.
Serves the web dashboard and bridges ROS topics via WebSocket.
"""
import os
import sys
import math
import time
import json
import atexit
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO

# Add the bonicbot-bridge root to Python path to import existing bridge controllers
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from bonicbot_bridge.core import BonicBot

# ── Flask & SocketIO Initialization ──────────────────────────────────────────
# Mandatory: async_mode='threading' to avoid conflict with roslibpy's Twisted thread
app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

# ── ROS Bridge Initialization ────────────────────────────────────────────────
print("⏳ Starting BonicBot ROS bridge connection...")
try:
    bot = BonicBot(host='localhost', port=9090, timeout=3.0)
    system = bot.system
    motion = bot.motion
except Exception as e:
    print(f"⚠️ WARNING: Could not connect to ROS bridge at localhost:9090. {e}")
    print("   Dashboard will load but no live data will stream.")
    # Initialize dummy controllers so the app doesn't crash on method calls
    # (In a real scenario, you'd want better fallback handling)
    bot = None
    system = None
    motion = None

def _shutdown():
    print("🛑 Shutting down BonicBot Dashboard...")
    if bot:
        bot.disconnect()

atexit.register(_shutdown)

# ── Subscribers & Callbacks ──────────────────────────────────────────────────
# All emits must use namespace='/' when called from a background thread

# Sentinel for change-detection on /map callbacks
_last_map_geo: dict = {}   # keys: width, height, resolution, origin_x, origin_y


def _on_map(msg: dict) -> None:
    try:
        global _last_map_geo
        info = msg.get('info', {})
        resolution = info.get('resolution', 0.05)
        width = info.get('width', 0)
        height = info.get('height', 0)
        origin = info.get('origin', {})
        data = msg.get('data', [])

        ts = int(time.time() * 1000)

        # Always send the full map.  SLAM Toolbox already rate-limits /map
        # via map_update_interval; no need to duplicate that with an
        # expensive hash(tuple(data)) on 100K+ elements.
        socketio.emit('map_update', {
            'ts': ts,
            'resolution': resolution,
            'width': width,
            'height': height,
            'data': data,
            'origin': origin,
        }, namespace='/')
    except Exception as exc:
        print(f"Error in _on_map: {exc}")

def euler_from_quaternion(x, y, z, w):
    """Convert a quaternion into euler angles (roll, pitch, yaw)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

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

def _on_locations_list(msg):
    try:
        data_str = msg.get('data', '[]')
        locations = json.loads(data_str)
        socketio.emit('locations_list', {'data': locations}, namespace='/')
    except Exception as e:
        print(f"Error parsing locations: {e}")
        socketio.emit('locations_list', {'data': []}, namespace='/')

def _on_current_goal(msg):
    try:
        pos = msg.get('pose', {}).get('position', {})
        socketio.emit('current_goal', {'x': pos.get('x', 0.0), 'y': pos.get('y', 0.0)}, namespace='/')
    except Exception as e:
        pass

# Simple forwarder generators
def make_forwarder(event_name):
    def _cb(msg):
        socketio.emit(event_name, msg, namespace='/')
    return _cb

# Subscribe via SystemController wrappers
if system and motion:
    system.subscribe_to_map(_on_map, throttle_rate=500)
    system.subscribe_to_odom(_on_odom, throttle_rate=100)
    system.subscribe_to_robot_state(make_forwarder('robot_state'))
    system.subscribe_to_mapping_active(make_forwarder('mapping_active'))
    system.subscribe_to_navigation_active(make_forwarder('navigation_active'))
    system.subscribe_to_current_goal(_on_current_goal, throttle_rate=500)
    motion.subscribe_to_distance_to_goal(make_forwarder('distance_to_goal'))
    system.subscribe_to_locations_list(_on_locations_list, throttle_rate=1000)
    system.subscribe_to_map_available(make_forwarder('map_available'))
    motion.subscribe_to_nav_status(make_forwarder('nav_status'))


# ── Web Routes & WebSocket Handlers ──────────────────────────────────────────

@socketio.on('connect')
def handle_connect():
    if not system or not motion:
        return
    # Emit initial state so UI is correctly populated on page load
    socketio.emit('robot_state', {'data': system.get_robot_state()}, namespace='/')
    socketio.emit('mapping_active', {'data': system.is_mapping()}, namespace='/')
    socketio.emit('navigation_active', {'data': system.is_navigating()}, namespace='/')
    socketio.emit('nav_status', {'data': motion.get_nav_status()}, namespace='/')

@app.route('/')
def index():
    is_conn = bot.is_connected() if bot else False
    return render_template('index.html', connected=is_conn)


# ── REST Endpoints: Mapping ──────────────────────────────────────────────────

@app.route('/api/mapping/start', methods=['POST'])
def mapping_start():
    try:
        system.start_mapping()
        return jsonify({'success': True, 'message': 'Mapping started'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/mapping/stop', methods=['POST'])
def mapping_stop():
    try:
        system.stop_mapping()
        return jsonify({'success': True, 'message': 'Mapping stopped'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/mapping/save', methods=['POST'])
def mapping_save():
    try:
        system.save_map()
        return jsonify({'success': True, 'message': 'Map saved'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


# ── REST Endpoints: Navigation ───────────────────────────────────────────────

@app.route('/api/navigation/start', methods=['POST'])
def nav_start():
    try:
        success = system.start_navigation(force=True)
        if not success:
            return jsonify({'success': False, 'message': 'Failed to start navigation'}), 400
        return jsonify({'success': True, 'message': 'Navigation started'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/stop', methods=['POST'])
def nav_stop():
    try:
        success = system.stop_navigation()
        if not success:
            return jsonify({'success': False, 'message': 'Failed to stop navigation (or not active)'}), 400
        return jsonify({'success': True, 'message': 'Navigation stopped'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/goal', methods=['POST'])
def nav_goal():
    try:
        data = request.json
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        theta_deg = float(data.get('theta', 0.0))
        
        success = motion.go_to(x, y, theta_deg)
        if not success:
            return jsonify({'success': False, 'message': 'Navigation is not active. Start Nav2 first.'}), 400
            
        return jsonify({'success': True, 'message': f'Goal sent: ({x:.2f}, {y:.2f})'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/cancel', methods=['POST'])
def nav_cancel():
    try:
        cancelled = motion.cancel_goal()
        if cancelled:
            return jsonify({'success': True, 'message': 'Goal cancelled'})
        else:
            return jsonify({'success': True, 'message': 'No active goal to cancel'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/initial_pose', methods=['POST'])
def nav_initial_pose():
    try:
        data = request.json
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        theta_deg = float(data.get('theta', 0.0))
        
        motion.set_initial_pose(x, y, theta_deg)
        return jsonify({'success': True, 'message': f'Initial pose set at ({x:.2f}, {y:.2f})'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/goto', methods=['POST'])
def nav_goto():
    try:
        data = request.json
        name = data.get('name', '')
        system.goto_location(name)
        return jsonify({'success': True, 'message': f'Going to {name}'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/save_location', methods=['POST'])
def nav_save_loc():
    try:
        data = request.json
        name = data.get('name', '')
        system.save_location(name)
        return jsonify({'success': True, 'message': f'Location {name} saved'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/delete_location', methods=['POST'])
def nav_del_loc():
    try:
        data = request.json
        name = data.get('name', '')
        system.delete_location(name)
        return jsonify({'success': True, 'message': f'Location {name} deleted'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


if __name__ == '__main__':
    print("🚀 Starting Flask server on http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)
