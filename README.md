# BonicBot Bridge 🤖

[![PyPI version](https://badge.fury.io/py/bonicbot-bridge.svg)](https://badge.fury.io/py/bonicbot-bridge)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**BonicBot Bridge** is a Python SDK for educational robotics programming with the BonicBot robots. It provides a simple, intuitive API that abstracts the complexity of ROS2 robotics into easy-to-use commands perfect for STEM education.

## 🚀 Quick Start

### Installation

```bash
pip install bonicbot-bridge
```

### Basic Usage

```python
from bonicbot_bridge import BonicBot

# Connect to robot (automatically finds robot on network)
bot = BonicBot()

# Basic movement
bot.move_forward(speed=0.3, duration=2)
bot.turn_left(speed=0.5, duration=1)
bot.stop()

# Sensors
position = bot.get_position()
print(f"Robot is at: {position}")

# Servo control
bot.move_left_arm(90, 30)  # shoulder, elbow angles
bot.look_left()
bot.open_grippers()

# Disconnect
bot.disconnect()
```

### Context Manager (Recommended)

```python
with BonicBot() as bot:
    bot.move_forward(0.3, duration=2)
    bot.turn_right(0.5, duration=1)
    # Automatically disconnects when done
```

## 📋 Features

- 🎯 **Simple API**: Easy-to-understand commands for educational use
- 🌐 **Remote Control**: Control robot from any computer on the network
- 🗺️ **SLAM Mapping**: Create and save maps of the environment
- 🧭 **Autonomous Navigation**: Navigate to specific coordinates
- 📊 **Sensor Access**: Read position, battery, and other sensor data
- 📷 **Camera Streaming**: Real-time image capture and processing
- 🤖 **Servo Control**: Control robot arms, grippers, and neck
- 🔄 **Real-time Feedback**: Live updates on robot status and goals
- 🛡️ **Safety Features**: Built-in error handling and connection management
- 📚 **Educational Focus**: Designed specifically for STEM learning

## 📖 API Reference

### BonicBot Class

#### Constructor

```python
BonicBot(host='localhost', port=9090, timeout=10)
```

**Parameters:**

- `host` (str): Robot IP address or hostname. Use 'localhost' if running on robot, or robot's IP/hostname for remote access
- `port` (int): rosbridge_server port (default: 9090)
- `timeout` (int): Connection timeout in seconds

**Examples:**

```python
# Local connection (running on robot)
bot = BonicBot()

# Remote connection
bot = BonicBot(host='192.168.1.100')
bot = BonicBot(host='bonic.local')
```

### Movement Methods

#### `move(linear_x=0, linear_y=0, angular_z=0)`

Low-level velocity control for custom robot movement patterns. This method gives you full control over linear and angular velocities simultaneously.

**Parameters:**
- `linear_x` (float): Forward/backward velocity in m/s
- `linear_y` (float): Left/right velocity in m/s (for omnidirectional robots)
- `angular_z` (float): Rotational velocity in deg/s

**⚠️ Important**: Due to ROS2's `cmd_vel_timeout` (typically 0.5s), commands must be **continuously published** to maintain movement. For duration-based control, publish in a loop at 10Hz.

**Basic Usage:**

```python
# Simple forward movement (continuous until stopped)
bot.motion.move(linear_x=0.3)
# ... robot moves forward
bot.stop()  # Stop when done

# Pure rotation (spin in place)
bot.motion.move(angular_z=30.0)  # 30 deg/s
# ... robot spins
bot.stop()
```

**Duration Control Pattern:**

For timed movements, publish commands in a loop:

```python
import time

# Move forward for 3 seconds
start = time.time()
while (time.time() - start) < 3.0:
    bot.motion.move(linear_x=0.3)
    time.sleep(0.1)  # Publish at 10Hz
bot.stop()
```

**Advanced Patterns:**

```python
# Circular arc (forward + rotation)
start = time.time()
while (time.time() - start) < 5.0:
    bot.motion.move(linear_x=0.2, angular_z=20.0)  # Drive in circle
    time.sleep(0.1)
bot.stop()

# Figure-8 pattern
# Left arc
start = time.time()
while (time.time() - start) < 2.5:
    bot.motion.move(linear_x=0.2, angular_z=30.0)
    time.sleep(0.1)

# Right arc
start = time.time()
while (time.time() - start) < 2.5:
    bot.motion.move(linear_x=0.2, angular_z=-30.0)
    time.sleep(0.1)

bot.stop()
```

**💡 Tip**: For simple forward/backward/turn movements with automatic duration control, use the convenience methods (`move_forward()`, `turn_left()`, etc.) instead. They handle the continuous publishing automatically.

#### `move_forward(speed, duration=None)`

Move robot forward at specified speed.

```python
bot.move_forward(0.3)           # Move forward at 0.3 m/s continuously
bot.move_forward(0.5, 2.0)      # Move forward for 2 seconds
```

#### `move_backward(speed, duration=None)`

Move robot backward at specified speed.

```python
bot.move_backward(0.2, 1.5)     # Move backward for 1.5 seconds
```

#### `turn_left(speed, duration=None)`

Turn robot left (counter-clockwise).

```python
bot.turn_left(0.5, 1.0)         # Turn left for 1 second
```

#### `turn_right(speed, duration=None)`

Turn robot right (clockwise).

```python
bot.turn_right(0.5, 1.0)        # Turn right for 1 second
```

#### `stop()`

Stop all robot movement immediately.

```python
bot.stop()
```

### Navigation Methods

#### `start_navigation()`

Start the navigation system (required before using navigation commands).

```python
bot.start_navigation()
```

#### `stop_navigation()`

Stop the navigation system.

```python
bot.stop_navigation()
```

#### `go_to(x, y, theta=0)`

Navigate to specific coordinates autonomously.

**Parameters:**

- `x` (float): Target X coordinate in meters
- `y` (float): Target Y coordinate in meters
- `theta` (float): Target orientation in degrees (optional)

```python
bot.go_to(2.0, 1.5)             # Navigate to (2.0, 1.5)
bot.go_to(0, 0, 90)             # Go to origin facing 90 degrees
```

#### `wait_for_goal(timeout=30)`

Wait for current navigation goal to complete.

**Returns:** Navigation result ('goal_reached', 'goal_failed', 'cancelled', or 'timeout')

```python
result = bot.wait_for_goal()
if result == 'goal_reached':
    print("Successfully reached destination!")
```

#### `cancel_goal()`

Cancel current navigation goal.

```python
bot.cancel_goal()
```

#### `set_initial_pose(x, y, theta=0)`

Set the robot's initial pose for localization on a map.

**Parameters:**
- `x` (float): Initial X coordinate in meters
- `y` (float): Initial Y coordinate in meters  
- `theta` (float): Initial orientation in degrees (optional)

```python
# Set robot at origin
bot.set_initial_pose(0.0, 0.0, 0.0)

# Set with specific orientation (90 degrees)
bot.set_initial_pose(2.0, 1.5, 90)
```

### Sensor Methods

#### `get_position()`

Get current robot position and orientation.

**Returns:** Dict with keys 'x', 'y', 'theta' (degrees) or None if no data available

```python
pos = bot.get_position()
if pos:
    print(f"X: {pos['x']:.2f}, Y: {pos['y']:.2f}, Heading: {pos['theta']:.2f}°")
```

#### `get_x()`, `get_y()`, `get_heading()`

Get individual position components.

```python
x = bot.get_x()                 # Current X position
y = bot.get_y()                 # Current Y position
heading = bot.get_heading()     # Current heading in degrees
```

#### `get_heading_degrees()`

Get current heading in degrees.

```python
heading_deg = bot.get_heading_degrees()  # Same as get_heading()
```

#### `get_battery()`

Get battery level percentage (0-100).

```python
battery = bot.get_battery()
print(f"Battery: {battery}%")
```

### System Control Methods

#### `start_mapping()`

Start SLAM (Simultaneous Localization and Mapping) mode.

```python
bot.start_mapping()
# Drive around to create map
bot.save_map()
```

#### `stop_mapping()`

Stop SLAM mapping mode.

```python
bot.stop_mapping()
```

#### `save_map()`

Save the current map created during mapping.

```python
bot.save_map()
```

### Status Methods

#### `get_nav_status()`

Get current navigation status.

**Returns:** Status string ('idle', 'navigating', 'goal_reached', 'goal_failed', 'cancelled')

#### `get_distance_to_goal()`

Get distance to current navigation goal in meters.

```python
distance = bot.get_distance_to_goal()
print(f"Distance remaining: {distance:.1f}m")
```

#### `is_connected()`

Check if connected to robot.

```python
if bot.is_connected():
    print("Robot connection OK")
```

### Camera Methods

#### `start_camera(callback=None)`

Start camera streaming with optional callback for real-time processing.

**Parameters:**
- `callback` (function): Optional function called for each frame: `callback(image)`

```python
# Simple streaming
bot.start_camera()

# With callback for processing
def process_frame(image):
    print(f"Frame: {image.shape}")
    
bot.start_camera(callback=process_frame)
```

#### `stop_camera()`

Stop camera streaming.

```python
bot.stop_camera()
```

#### `get_image()`

Get the latest camera image as numpy array (BGR format).

**Returns:** numpy.ndarray or None

```python
image = bot.get_image()
if image is not None:
    print(f"Image shape: {image.shape}")
```

#### `save_image(filepath)`

Save current camera image to file.

```python
bot.save_image("robot_view.jpg")
```

### Servo Control Methods

#### `set_servos(angles)`

Set multiple servo angles at once.

**Parameters:**
- `angles` (dict): Dictionary mapping joint names to angles in degrees

```python
bot.set_servos({
    'left_shoulder_pitch_joint': 45.0,
    'neck_yaw_joint': -30.0
})
```

#### `move_left_arm(shoulder, elbow)` / `move_right_arm(shoulder, elbow)`

Move robot arms to specified angles.

**Parameters:**
- `shoulder` (float): Shoulder pitch angle (-45° to 180°)
- `elbow` (float): Elbow angle (0° to 50°)

```python
bot.move_left_arm(90, 30)   # Left arm up
bot.move_right_arm(45, 20)  # Right arm halfway
```

#### `set_grippers(left, right)` / `open_grippers()` / `close_grippers()`

Control gripper fingers.

```python
bot.open_grippers()         # Open both grippers
bot.close_grippers()        # Close both grippers
bot.set_grippers(30, 30)    # Partial open
```

#### `set_neck(yaw)` / `look_left()` / `look_right()` / `look_center()`

Control neck rotation.

**Parameters:**
- `yaw` (float): Neck yaw angle (-90° to 90°)

```python
bot.set_neck(-45)   # Look right 45°
bot.look_left()     # Turn fully left
bot.look_center()   # Center position
```

#### `reset_servos()`

Reset all servos to neutral position (0°).

```python
bot.reset_servos()
```

### Example 5: Camera Vision

```python
from bonicbot_bridge import BonicBot
import cv2

with BonicBot() as bot:
    # Start camera with callback
    def detect_objects(image):
        # Simple color detection example
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Detect red objects
        lower_red = (0, 100, 100)
        upper_red = (10, 255, 255)
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        if cv2.countNonZero(mask) > 1000:
            print("Red object detected!")
    
    bot.start_camera(callback=detect_objects)
    
    # Let it run for 10 seconds
    import time
    time.sleep(10)
    
    # Save a snapshot
    bot.save_image("detection_result.jpg")
    bot.stop_camera()
```

### Example 6: Servo Gestures

```python
from bonicbot_bridge import BonicBot
import time

with BonicBot() as bot:
    print("Robot greeting sequence...")
    
    # Wave hello
    bot.look_center()
    bot.servo.wave_right_arm(duration=3)
    time.sleep(0.5)
    
    # Look around
    bot.look_left()
    time.sleep(1)
    bot.look_right()
    time.sleep(1)
    bot.look_center()
    
    # Gripper demo
    bot.move_left_arm(90, 30)
    bot.move_right_arm(90, 30)
    bot.open_grippers()
    time.sleep(1)
    bot.close_grippers()
    
    # Reset
    bot.reset_servos()
    print("Greeting complete!")
```
## 🎓 Educational Examples

### Example 1: Basic Movement

```python
from bonicbot_bridge import BonicBot
import time

with BonicBot() as bot:
    print("Drawing a square...")

    for i in range(4):
        bot.move_forward(0.3, duration=2)   # Move forward
        bot.turn_left(0.5, duration=1.6)    # Turn 90 degrees
        time.sleep(0.5)                     # Pause between moves

    print("Square complete!")
```

### Example 2: Sensor Data Collection

```python
from bonicbot_bridge import BonicBot
import time

with BonicBot() as bot:
    print("Collecting position data...")

    positions = []

    # Move forward while collecting data
    bot.move_forward(0.2)

    for i in range(10):
        pos = bot.get_position()
        if pos:
            positions.append(pos)
            print(f"Position {i}: X={pos['x']:.2f}, Y={pos['y']:.2f}")
        time.sleep(0.5)

    bot.stop()
    print(f"Collected {len(positions)} data points")
```

### Example 3: Autonomous Navigation

```python
from bonicbot_bridge import BonicBot

with BonicBot() as bot:
    # Start navigation system
    bot.start_navigation()

    # Define waypoints for a patrol route
    waypoints = [
        (2.0, 0.0),
        (2.0, 2.0),
        (0.0, 2.0),
        (0.0, 0.0)
    ]

    for i, (x, y) in enumerate(waypoints):
        print(f"Going to waypoint {i+1}: ({x}, {y})")
        bot.go_to(x, y)

        result = bot.wait_for_goal(timeout=30)
        if result == 'goal_reached':
            print(f"Reached waypoint {i+1}")
        else:
            print(f"Failed to reach waypoint {i+1}: {result}")
            break

    print("Patrol complete!")
```

### Example 4: Mapping and Navigation

```python
from bonicbot_bridge import BonicBot
import time

with BonicBot() as bot:
    print("Creating map of environment...")

    # Start mapping
    bot.start_mapping()

    # Explore the area (manual or programmed exploration)
    exploration_moves = [
        ('forward', 2),
        ('left', 1),
        ('forward', 2),
        ('right', 2),
        ('forward', 2)
    ]

    for move_type, duration in exploration_moves:
        if move_type == 'forward':
            bot.move_forward(0.3, duration)
        elif move_type == 'left':
            bot.turn_left(0.5, duration)
        elif move_type == 'right':
            bot.turn_right(0.5, duration)

        time.sleep(1)  # Pause between moves

    # Save the map
    bot.save_map()
    print("Map saved!")

    # Now start navigation with the created map
    bot.start_navigation()

    # Navigate back to start
    bot.go_to(0, 0)
    bot.wait_for_goal()
    print("Returned to starting position!")
```

## 🔧 Advanced Usage

### Custom Callbacks

```python
from bonicbot_bridge import BonicBot

def position_callback(x, y, theta):
    print(f"Robot moved to: ({x:.2f}, {y:.2f})")

bot = BonicBot()
bot.sensors.subscribe_to_position(position_callback)
```

### Error Handling

```python
from bonicbot_bridge import BonicBot, ConnectionError, NavigationError

try:
    with BonicBot(host='192.168.1.100') as bot:
        bot.start_navigation()
        bot.go_to(5, 5)

except ConnectionError as e:
    print(f"Could not connect to robot: {e}")

except NavigationError as e:
    print(f"Navigation failed: {e}")
```

### Integration with Other Libraries

```python
from bonicbot_bridge import BonicBot
import numpy as np
import matplotlib.pyplot as plt
import time

with BonicBot() as bot:
    # Collect position data
    positions = []

    bot.move_forward(0.2)

    for i in range(50):
        pos = bot.get_position()
        if pos:
            positions.append([pos['x'], pos['y']])
        time.sleep(0.1)

    bot.stop()

    # Plot trajectory using matplotlib
    if positions:
        trajectory = np.array(positions)
        plt.figure(figsize=(8, 6))
        plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2)
        plt.scatter(trajectory[0, 0], trajectory[0, 1], color='green', s=100, label='Start')
        plt.scatter(trajectory[-1, 0], trajectory[-1, 1], color='red', s=100, label='End')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Robot Trajectory')
        plt.grid(True)
        plt.legend()
        plt.show()
```

## 🛠️ Technical Details

### System Requirements

- **Python**: 3.8 or higher
- **Robot**: BonicBot A2 with ROS2 Humble
- **Network**: Robot and computer must be on same network (for remote control)
- **Dependencies**: roslibpy (automatically installed)

### Supported Platforms

- **Raspberry Pi 4** (recommended for onboard execution)
- **Ubuntu 20.04/22.04**
- **Windows 10/11** (for remote control)
- **macOS** (for remote control)

### ROS2 Topic Integration

The library communicates with these ROS2 topics and services:

**Topics:**

- `/cmd_vel` (geometry_msgs/Twist) - Robot movement commands
- `/diff_cont/odom` (nav_msgs/Odometry) - Robot position feedback
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goals
- `/robot/nav_status` (std_msgs/String) - Navigation status updates
- `/robot/distance_to_goal` (std_msgs/Float32) - Distance feedback
- `/joint_states` (sensor_msgs/JointState) - Servo position feedback
- `/servo_position_controller/commands` (std_msgs/Float64MultiArray) - Servo commands
- `/camera/image_raw/compressed` (sensor_msgs/CompressedImage) - Camera images
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Camera metadata
- `/robot/camera_active` (std_msgs/Bool) - Camera status

**Services:**

- `/robot/start_mapping` (std_srvs/Trigger) - Start SLAM mapping
- `/robot/stop_mapping` (std_srvs/Trigger) - Stop SLAM mapping
- `/robot/save_map` (std_srvs/Trigger) - Save current map
- `/robot/start_navigation` (std_srvs/Trigger) - Start navigation
- `/robot/stop_navigation` (std_srvs/Trigger) - Stop navigation
- `/robot/cancel_navigation` (std_srvs/Trigger) - Cancel current goal
- `/robot/start_camera` (std_srvs/Trigger) - Start camera system
- `/robot/stop_camera` (std_srvs/Trigger) - Stop camera system

### Performance Tips

1. **Connection Management**: Use context managers (`with BonicBot() as bot:`) for automatic cleanup
2. **Remote Latency**: For remote control, expect 10-50ms latency depending on network
3. **Sensor Updates**: Position data updates at ~20Hz, battery at ~1Hz
4. **Goal Setting**: Wait for previous navigation goals to complete before setting new ones

## 🐛 Troubleshooting

### Common Issues

**Connection Failed**

```
ConnectionError: Failed to connect to robot at localhost:9090
```

- Ensure rosbridge_server is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check network connectivity: `ping bonic.local`
- Verify port 9090 is open and not blocked by firewall

**Navigation Not Working**

```
NavigationError: Failed to start navigation: No saved map found
```

- Create a map first using `bot.start_mapping()` and `bot.save_map()`
- Or start mapping and navigation together: `bot.system.quick_map_and_nav()`

**Import Error**

```
ModuleNotFoundError: No module named 'bonicbot_bridge'
```

- Install the library: `pip install bonicbot-bridge`
- For development: `pip install -e .` from the source directory

### Debug Mode

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Now BonicBot will show detailed connection and command logs
bot = BonicBot()
```

## 🤝 Contributing

This is a commercial library maintained by Autobonics Pvt Ltd. For bug reports, feature requests, or support:

- **Email**: support@bonic.ai
- **Documentation**: https://docs.bonic.ai/
- **Website**: https://bonic.ai/

## 📄 License

Copyright (c) 2024 Autobonics Pvt Ltd. All rights reserved.

This software is licensed under a commercial license. Educational institutions may use this library free of charge with BonicBot robots. For commercial licensing inquiries, contact licensing@bonic.ai.

## 🙏 Acknowledgments

- Built on top of [ROS2](https://docs.ros.org/en/humble/) and [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
- Uses [roslibpy](https://github.com/gramaziokohler/roslibpy) for WebSocket communication
- Designed for [BonicBot A2](https://bonic.ai/products/bonicbot-a2) educational robot

---

**Made with ❤️ for STEM Education by [Autobonics](https://bonic.ai/)**

## 🧪 Example Scripts

The library includes ready-to-use example scripts in the `examples/` directory:

### Camera Test

Test camera streaming and image capture:

```bash
python3 examples/test_camera.py --host <robot_ip>
```

**Features:**
- Start/stop camera service
- Stream compressed images  
- Display camera info (resolution, distortion model)
- Save snapshots to file

### Servo Test

Test all servo motors (arms, grippers, neck):

```bash
python3 examples/test_servos.py --host <robot_ip>
```

**Features:**
- Test each servo individually
- Display servo limits
- Wave arm demonstration
- Gripper open/close test
- Neck rotation test

### Servo Monitor

Real-time display of servo positions:

```bash
python3 examples/monitor_servos.py --host <robot_ip> --rate 0.2
```

**Features:**
- Live servo angle display
- Configurable update rate
- Monitor all 7 servos simultaneously
- Press Ctrl+C to exit

### Integrated Demo

Complete demonstration of all features:

```bash
python3 examples/demo_integrated.py --host <robot_ip>
```

**Features:**
- Camera + servo + navigation
- Automated test sequence
- Multiple snapshot capture
- Movement with servo gestures

