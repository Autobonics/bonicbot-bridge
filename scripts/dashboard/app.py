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

import roslibpy
from bonicbot_bridge.system import SystemController
from bonicbot_bridge.motion import MotionController
from bonicbot_bridge.utils import (
    STRING_MESSAGE_TYPE,
    BOOL_MESSAGE_TYPE,
    POSE_STAMPED_MESSAGE_TYPE,
    FLOAT32_MESSAGE_TYPE,
    ODOMETRY_MESSAGE_TYPE,
)

OCCUPANCY_GRID_MESSAGE_TYPE = 'nav_msgs/OccupancyGrid'

# ── Flask & SocketIO Initialization ──────────────────────────────────────────
# Mandatory: async_mode='threading' to avoid conflict with roslibpy's Twisted thread
app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

# ── ROS Bridge Initialization ────────────────────────────────────────────────
ros = roslibpy.Ros(host='localhost', port=9090)

print("⏳ Starting ROS bridge background thread...")
ros.run()

# Wait briefly to see if connection succeeds
time.sleep(1.0)
if not ros.is_connected:
    print("⚠️ WARNING: Could not connect to ROS bridge at localhost:9090.")
    print("   Dashboard will load but no live data will stream.")

# Instantiate BonicBot Controllers
system = SystemController(ros)
motion = MotionController(ros)
system._motion = motion
motion.system = system

def _shutdown():
    print("🛑 Shutting down BonicBot Dashboard...")
    motion.shutdown()
    system.shutdown()
    try:
        ros.terminate()
    except Exception as e:
        pass

atexit.register(_shutdown)

# ── Publishers ───────────────────────────────────────────────────────────────
goto_loc_pub = roslibpy.Topic(ros, '/robot/goto_location', STRING_MESSAGE_TYPE)
save_loc_pub = roslibpy.Topic(ros, '/robot/save_location', STRING_MESSAGE_TYPE)
delete_loc_pub = roslibpy.Topic(ros, '/robot/delete_location', STRING_MESSAGE_TYPE)

# Advertise publishers
goto_loc_pub.advertise()
save_loc_pub.advertise()
delete_loc_pub.advertise()

# ── Subscribers & Callbacks ──────────────────────────────────────────────────
# All emits must use namespace='/' when called from a background thread

# Sentinel for change-detection on /map callbacks
_map_geometry: dict = {}   # keys: width, height, resolution, origin_x, origin_y
_map_data_hash: int = 0    # hash of the flat data array for cheap equality check


def _on_map(msg: dict) -> None:
    try:
        global _map_geometry, _map_data_hash
        info = msg.get('info', {})
        resolution = info.get('resolution', 0.05)
        width = info.get('width', 0)
        height = info.get('height', 0)
        origin = info.get('origin', {})
        data = msg.get('data', [])

        origin_pos = origin.get('position', {})
        geo = {
            'width': width,
            'height': height,
            'resolution': resolution,
            'origin_x': origin_pos.get('x', 0.0),
            'origin_y': origin_pos.get('y', 0.0),
        }

        new_hash = hash(tuple(data))
        ts = int(time.time() * 1000)

        if geo != _map_geometry or new_hash != _map_data_hash:
            _map_geometry = geo
            _map_data_hash = new_hash
            socketio.emit('map_update', {
                'ts': ts,
                'resolution': resolution,
                'width': width,
                'height': height,
                'data': data,
                'origin': origin,
            }, namespace='/')
        else:
            socketio.emit('map_tick', {'ts': ts}, namespace='/')
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

# Define Topics
subscribers = [
    roslibpy.Topic(ros, '/map', OCCUPANCY_GRID_MESSAGE_TYPE, throttle_rate=1000),
    roslibpy.Topic(ros, '/diff_cont/odom', ODOMETRY_MESSAGE_TYPE, throttle_rate=100),
    roslibpy.Topic(ros, '/robot/state', STRING_MESSAGE_TYPE, throttle_rate=500),
    roslibpy.Topic(ros, '/robot/mapping_active', BOOL_MESSAGE_TYPE, throttle_rate=500),
    roslibpy.Topic(ros, '/robot/navigation_active', BOOL_MESSAGE_TYPE, throttle_rate=500),
    roslibpy.Topic(ros, '/robot/current_goal', POSE_STAMPED_MESSAGE_TYPE, throttle_rate=500),
    roslibpy.Topic(ros, '/robot/distance_to_goal', FLOAT32_MESSAGE_TYPE, throttle_rate=500),
    roslibpy.Topic(ros, '/robot/locations_list', STRING_MESSAGE_TYPE, throttle_rate=1000),
    roslibpy.Topic(ros, '/robot/map_available', BOOL_MESSAGE_TYPE, throttle_rate=1000),
    roslibpy.Topic(ros, '/robot/nav_status', STRING_MESSAGE_TYPE, throttle_rate=200),
]

# Subscribe
subscribers[0].subscribe(_on_map)
subscribers[1].subscribe(_on_odom)
subscribers[2].subscribe(make_forwarder('robot_state'))
subscribers[3].subscribe(make_forwarder('mapping_active'))
subscribers[4].subscribe(make_forwarder('navigation_active'))
subscribers[5].subscribe(_on_current_goal)
subscribers[6].subscribe(make_forwarder('distance_to_goal'))
subscribers[7].subscribe(_on_locations_list)
subscribers[8].subscribe(make_forwarder('map_available'))
subscribers[9].subscribe(make_forwarder('nav_status'))


# ── Web Routes ───────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html', connected=ros.is_connected)


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
        system.start_navigation()
        return jsonify({'success': True, 'message': 'Navigation started'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/stop', methods=['POST'])
def nav_stop():
    try:
        system.stop_navigation()
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
        goto_loc_pub.publish({'data': name})
        return jsonify({'success': True, 'message': f'Going to {name}'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/save_location', methods=['POST'])
def nav_save_loc():
    try:
        data = request.json
        name = data.get('name', '')
        save_loc_pub.publish({'data': name})
        return jsonify({'success': True, 'message': f'Location {name} saved'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/navigation/delete_location', methods=['POST'])
def nav_del_loc():
    try:
        data = request.json
        name = data.get('name', '')
        delete_loc_pub.publish({'data': name})
        return jsonify({'success': True, 'message': f'Location {name} deleted'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


if __name__ == '__main__':
    print("🚀 Starting Flask server on http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)
