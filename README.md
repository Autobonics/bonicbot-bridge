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
- 👁️ **Vision & AI Detection**: Object detection, face detection, pose estimation, gesture recognition, and ArUco marker tracking
- 📚 **Educational Focus**: Designed specifically for STEM learning

## 📖 API Reference

### BonicBot Class

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **CONSTRUCTOR**

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
```

---

### 🚗 Movement Methods

#### `move(linear_x=0, linear_y=0, angular_z=0)`

Low-level velocity control for custom robot movement patterns. This method gives you full control over linear and angular velocities simultaneously.

**Parameters:**
- `linear_x` (float): Forward/backward velocity in m/s
- `linear_y` (float): Left/right velocity in m/s (for omnidirectional robots)
- `angular_z` (float): Rotational velocity in deg/s

> [!IMPORTANT]
> Due to ROS2's `cmd_vel_timeout` (typically 0.5s), commands must be **continuously published** to maintain movement. For duration-based control, publish in a loop at 10Hz.

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

> [!TIP]
> For simple forward/backward/turn movements with automatic duration control, use the convenience methods (`move_forward()`, `turn_left()`, etc.) instead. They handle the continuous publishing automatically.

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

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **PRECISE MOTION CONTROL**

For exact movements with closed-loop odometry feedback, use the precise motion methods.

##### `drive_distance(dist, speed=0.3, engine='internal', timeout=30.0)`
Drive a specific distance in meters.
- `dist`: Distance to travel in meters (negative for backward). **Max 2.0 m** per call (`MAX_PRECISE_DISTANCE`).
- `engine`: Engine to use. `'internal'` uses odometry directly, `'nav2'` uses action servers.

> [!IMPORTANT]
> A single call is limited to 2.0 m (`MAX_PRECISE_DISTANCE` in `utils.py`) to prevent accidental long-distance drives.

```python
bot.drive_distance(1.0)        # Drive forward 1 meter
bot.drive_distance(-0.5)       # Drive backward 0.5 meters
```

##### `rotate_angle(angle, speed=45.0, engine='internal', timeout=30.0)`
Rotate exactly by a given angle in degrees.
- `angle`: Rotation in degrees (positive for left/CCW, negative for right/CW).

```python
bot.rotate_angle(90)           # Turn exactly 90 degrees left
```

##### `drive_and_rotate(dist, angle, speed=0.3, turn_speed=45.0, engine='internal', timeout=30.0)`
Perform a drive followed by a rotation in a single call.

```python
bot.drive_and_rotate(1.0, 90)  # Drive 1m, then turn 90° left
```

##### `set_default_engine(engine)` & `PreciseMotionEngine` enum
Switch the default engine used for precise movements.
Valid options are `'internal'` (default) or `'nav2'`. The `PreciseMotionEngine` enum can also be used.

```python
from bonicbot_bridge.precisemotion import PreciseMotionEngine
bot.set_default_engine(PreciseMotionEngine.NAV2)
```

##### `is_precise_moving()`
Check if a precise movement operation is currently in progress.

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **COMMAND QUEUES**

You can queue multiple precise movements to be executed sequentially.

##### `enqueue_move(cmd_list)`
Push a list of movement commands into the queue. Each command is a dictionary.

```python
bot.enqueue_move([
    {'type': 'drive', 'value': 1.0, 'speed': 0.3},
    {'type': 'rotate', 'value': 90, 'speed': 45.0}
])
```

##### `run_queue(block=True)`
Execute all queued commands sequentially. Set `block=False` to run them asynchronously.

```python
bot.run_queue()
```

##### `clear_queue()`
Flush the queue and immediately stop any active queued movement.

##### `draw_square(side_m, speed=0.3, turn_speed=45.0, engine='internal', timeout=30.0)`
Convenience wrapper to automatically enqueue and execute a square pattern. `side_m` is **required** (no default).

```python
bot.draw_square(1.0)           # Drive a 1m square
bot.draw_square(0.5, speed=0.2) # Smaller, slower square
```


---

### 🧭 Navigation Methods

#### `start_navigation()`

Start the navigation system (required before using navigation commands).

> [!TIP]
> To force-start navigation without a saved map (useful for SLAM-mode exploration), use the system-level delegate: `bot.system.start_navigation(force=True)`.

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

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **AUTONOMOUS EXPLORATION**

The SDK provides an automated frontier-based exploration system (`explore_lite`) to map unknown environments.

##### `setup_for_exploration()`
A 7-step setup routine that fully prepares the robot for autonomous exploration. It starts SLAM, Navigation, the explore_lite server, and waits for the costmap to populate.

> [!TIP]
> To receive progress callbacks during setup, call the explore controller directly: `bot.explore.setup_for_exploration(progress_callback=lambda step: print(f"Setup: {step}"))`.

```python
bot.setup_for_exploration()
```

##### `start_explore()` / `stop_explore()`
Start or stop the exploration process.

```python
bot.start_explore()  # Robot begins driving to unknown frontiers
bot.stop_explore()
```

##### `is_exploring()`
Check if the robot is currently exploring.

##### `wait_for_map_complete(timeout=300.0)`
Block until the exploration node reports that all frontiers are exhausted (meaning the entire accessible area has been mapped).

> [!TIP]
> To receive progress callbacks, call the explore controller directly: `bot.explore.wait_for_map_complete(timeout=300.0, progress_callback=my_cb)`.

##### `explore.suspend_for_manual_control()` / `explore.resume_from_manual_control()`
Safely pause the autonomous exploration to manually control the robot (e.g. to nudge it out of a stuck position), and then resume exploration without losing state.

##### `explore.set_lifecycle_callback(callback)`
Attach a callback to listen to exploration lifecycle events (started, stopped, completed, failed, etc.).

##### `explore.diagnostics()`
Returns a dictionary containing a snapshot of the current internal exploration state.

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **NAMED LOCATION MANAGEMENT**

You can save and navigate to specific coordinates using friendly string names (e.g. "kitchen", "charging_dock").

##### `save_location(name)`
Save the robot's current pose under the given string name.

```python
bot.save_location("kitchen")
```

##### `goto_location(name)`
Autonomously navigate to a previously saved named location.

```python
bot.goto_location("kitchen")
bot.wait_for_goal()
```

##### `delete_location(name)` / `delete_all_locations()`
Remove one or all named locations.

```python
bot.delete_location("kitchen")
bot.delete_all_locations()
```

---

### 📡 Sensor Methods

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


#### `get_battery()`

Get battery level percentage (0-100).

```python
battery = bot.get_battery()
print(f"Battery: {battery}%")
```

#### `get_distance_traveled(start_pos=None)`

Calculate the distance traveled from a reference position. If `start_pos` is not provided, it uses the first received odometry position.

```python
dist = bot.get_distance_traveled()
print(f"Traveled: {dist:.2f}m")
```

#### `get_sensor_info()`

Returns a comprehensive dictionary containing the current state of all sensors (position, battery, heading, etc.).

#### `wait_for_data(timeout=5.0)`

Block until initial sensor data (like odometry) has been received from the robot. Useful to ensure the SDK is fully synced before issuing movement commands.

---

### ⚙️ System Control Methods

#### `setup_for_mapping()` / `setup_for_navigation()`
Convenience methods that ensure the system is correctly prepared for mapping or navigation. They activate the necessary hardware/sensors and wait for readiness.

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

bot.save_map()
```

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **MAP DATA ACCESS**

##### `has_saved_map()`
Check if a saved map exists on the robot's disk.

```python
if bot.has_saved_map():
    print("Robot has a map ready for navigation.")
```

##### `get_map_info()`
Get metadata about the current SLAM map. Returns a dictionary with `resolution` (m/cell), `width`, `height`, and `origin`.

##### `get_map_data()`
Get the full cached OccupancyGrid data as a dictionary. Note that this can be a large data structure.

---

### 📊 Status Methods

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

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **STATE CHECKERS**

Quickly verify the current state of various robot systems:
- `bot.is_moving()` — Checks if a navigation goal is actively being pursued.
- `bot.is_mapping()` — Check if SLAM is running.
- `bot.is_navigating()` — Check if the Navigation stack is active.

#### `get_robot_state()` / `get_system_status()`
- `get_robot_state()`: Returns the string state of the robot lifecycle (e.g. 'idle', 'mapping').
- `get_system_status()`: Returns a full dictionary snapshot of the entire system state (camera, vision, navigation, map).

---

### 📷 Camera Methods

> [!IMPORTANT]
> Camera operations have two parts:
1. **Hardware control** (server-side): Activates/deactivates physical camera
2. **Streaming control** (client-side): Subscribes/unsubscribes to camera images

**Recommended workflow:**

> [!NOTE]
> `bot.camera` is lazy-initialized and is only created on first access. Hardware control should be done via top-level `bot` or `bot.system` delegates.

```python
# 1. Activate camera hardware
bot.activate_camera_hardware()

# 2. Start receiving images
bot.start_camera()
bot.camera.wait_for_image(timeout=3.0)

# 3. Use camera
bot.save_image("photo.jpg")

# 4. Stop receiving images
bot.stop_camera()

# 5. Deactivate hardware (important for performance!)
bot.deactivate_camera_hardware()
```

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **HARDWARE CONTROL (SERVER-SIDE)**

##### `activate_camera_hardware()`

Activate the robot's physical camera hardware. Delegates to `system.start_camera()`.

```python
bot.activate_camera_hardware()  # Turn ON camera
```

##### `deactivate_camera_hardware()`

Deactivate camera hardware to free up resources. Delegates to `system.stop_camera()`.

```python
bot.deactivate_camera_hardware()  # Turn OFF camera
```

##### `system.is_camera_active()`

Check if camera hardware is currently activated.

```python
is_active = bot.system.is_camera_active()  # Returns True/False
```

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **STREAMING CONTROL (CLIENT-SIDE)**

##### `start_camera(callback=None)`

Start subscribing to camera images in your script.

**Parameters:**
- `callback` (function): Optional function called for each frame: `callback(image)`

```python
# Simple streaming
bot.start_camera()

# With callback for real-time processing
def process_frame(image):
    print(f"Frame: {image.shape}")
    
bot.start_camera(callback=process_frame)
```

##### `stop_camera()`

Stop subscribing to camera images.

```python
bot.stop_camera()
```

##### `camera.is_streaming()`

Check if currently receiving images.

```python
is_streaming = bot.camera.is_streaming()  # Returns True/False
```

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **IMAGE ACCESS**

##### `get_image()`

Get the latest camera image as numpy array (BGR format).

**Returns:** numpy.ndarray or None

```python
image = bot.get_image()
if image is not None:
    print(f"Image shape: {image.shape}")
```

##### `save_image(filepath)`

Save current camera image to file.

```python
bot.save_image("robot_view.jpg")
```

##### `camera.wait_for_image(timeout)` / `bot.wait_for_image(timeout)`

Wait for first image to arrive. Can be called directly on `bot`.

```python
bot.wait_for_image(timeout=5.0)  # Wait up to 5 seconds
```

##### `camera.get_camera_info()` / `bot.get_camera_info()`

Get camera metadata (resolution, distortion model, etc.). Can be called directly on `bot`.

```python
info = bot.get_camera_info()
print(f"Resolution: {info['width']}x{info['height']}")
```

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **COMPLETE EXAMPLE**

```python
with BonicBot(host='192.168.1.100') as bot:
    # Activate hardware
    bot.activate_camera_hardware()
    
    # Start receiving images
    bot.start_camera()
    bot.camera.wait_for_image(timeout=3.0)
    
    # Capture photo
    bot.save_image("destination.jpg")
    
    # Stop receiving
    bot.stop_camera()
    
    # Deactivate hardware
    bot.deactivate_camera_hardware()
```

---

### 🦾 Servo Control Methods

**Architecture**: The servo system uses separate ROS2 controller topics for each group (left arm, right arm, head, grippers).

---

#### `move_left_arm(shoulder, elbow, wait=True)` / `move_right_arm(shoulder, elbow, wait=True)`

Move robot arms to specified angles.

**Parameters:**
- `shoulder` (float): Shoulder pitch angle (-45° to 180°)
- `elbow` (float): Elbow angle (0° to 50°)
- `wait` (bool): Whether to block until the command is sent (default: True)

```python
bot.move_left_arm(90, 30)   # Left arm up
bot.move_right_arm(45, 20)  # Right arm halfway
```

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **GRIPPER CONTROL**

##### `set_grippers(left, right)`

Control both grippers simultaneously.

**Parameters:**
- `left` (float): Left gripper angle (-45° to 60°)
- `right` (float): Right gripper angle (-45° to 60°)

```python
bot.set_grippers(30, 30)    # Partial open
```

##### `set_left_gripper(angle)` / `set_right_gripper(angle)`

Control individual grippers independently. These are exposed as top-level delegates on `bot` as well as `bot.servo`.

**Parameters:**
- `angle` (float): Gripper angle (-45° to 60°)

```python
bot.set_left_gripper(30)   # Left gripper only
bot.set_right_gripper(45)  # Right gripper only
```

##### `open_grippers()` / `close_grippers()`

Convenience methods for both grippers.

```python
bot.open_grippers()         # Open both to 60°
bot.close_grippers()        # Close both to 0°
```

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **NECK CONTROL**

##### `set_neck(yaw)`

Control neck rotation.

**Parameters:**
- `yaw` (float): Neck yaw angle (-90° to 90°)

```python
bot.set_neck(-45)   # Look right 45°
bot.set_neck(0)     # Center
```

##### `look_left()` / `look_right()` / `look_center()`

Convenience methods for common neck positions.

```python
bot.look_left()     # Turn fully left (90°)
bot.look_right()    # Turn fully right (-90°)
bot.look_center()   # Center position (0°)
```

---

#### `reset_servos()`

Reset all servos to neutral position (0°).

```python
bot.reset_servos()
```

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **SERVO STATE & TOP-LEVEL DELEGATES**
- `bot.get_servo_angles()`: Returns a dict of all current joint angles.
- `bot.get_servo_limits()`: Returns a dict of safe (min, max) limits per joint.
- `bot.set_single_servo(joint, angle)` / `bot.get_single_servo(joint)`: Manage specific joints by string name.
- `bot.set_servos(angles_dict)`: Set multiple specific servos at once.

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **TECHNICAL DETAILS**

**ROS2 Topics:**
- `/left_arm_controller/commands` - [shoulder, elbow]
- `/right_arm_controller/commands` - [shoulder, elbow]
- `/head_controller/commands` - [yaw]
- `/left_gripper_controller/commands` - [finger1]
- `/right_gripper_controller/commands` - [finger1]

**Angle Limits:**
- Shoulder: -45° to 180°
- Elbow: 0° to 50°
- Gripper: -45° to 60°
- Neck: -90° to 90°

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
    bot.move_right_arm(90, 30)
    time.sleep(0.5)
    bot.move_right_arm(0, 0)
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

<details><summary>Click to expand examples</summary>

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

</details>

---

### 👁️ Vision & Detection Methods

> [!NOTE]
> **Architecture:** All vision inference runs on the robot's onboard `vision_pipeline.py` ROS 2 node. The bridge SDK simply sends configuration commands and receives detection results — no local models or GPU required on your machine.

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **DETECTION MODES**

The vision pipeline uses string literals to define all available pipeline modes.

| String | What it detects |
|--------|----------------|
| `'yolo'` | YOLO object detection (80 COCO classes: person, bottle, chair, etc.) |
| `'face'` | Human faces with 5 facial landmarks |
| `'pose'` | 33 MediaPipe body pose landmarks |
| `'gesture'` | Hand gestures (Thumb_Up, Open_Palm, Victory, etc.) |
| `'aruco'` | ArUco fiducial markers with full pose (tvec, rvec, distance) |

---

#### `enable_detection(mode, model='yolov8n')`

Enable a vision detection pipeline mode on the remote robot.

This calls the corresponding `/robot/enable_<mode>` service and publishes an update to `/vision/control`. If the vision pipeline is not already running, it is started automatically.

**Parameters:**
- `mode` (str): The detection mode to enable (`'yolo'`, `'face'`, `'pose'`, `'gesture'`, or `'aruco'`)
- `model` (str): YOLO model name (default: `'yolov8n'`). Reserved for future multi-model support

**Returns:** `dict` — the service response

**Raises:**
- `VisionError` if `mode` is not a valid detector name

```python
# YOLO object detection (uses robot's onboard yolov8n.onnx)
bot.enable_detection('yolo')

# Face detection
bot.enable_detection('face')

# Pose estimation
bot.enable_detection('pose')

# Gesture recognition
bot.enable_detection('gesture')

# ArUco marker tracking
bot.enable_detection('aruco')
```

---

#### `disable_detection()`

Disable the vision detection pipeline.

**Returns:** `True` on successful publish

```python
bot.disable_detection()
```

---

#### `get_active_mode()`

Return the currently active detector mode.

**Returns:** `str | None` — e.g. `'yolo'`, `'face'`, or `None` if no detector is active.

```python
status = bot.get_active_mode()
print(f"Active mode: {status}")
```

---

#### `get_active_detectors()`

Return a list of all currently active detector names.

**Returns:** `list[str]` — e.g. `['yolo', 'face']`

```python
active_detectors = bot.vision.get_active_detectors()
print(f"Active detectors: {active_detectors}")
```

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **STATUS PROPERTIES**

Check if specific detectors are currently active on the robot using properties on `bot.vision`:

- `bot.vision.vision_active` (bool): True if the vision pipeline is running.
- `bot.vision.yolo_enabled` (bool)
- `bot.vision.pose_enabled` (bool)
- `bot.vision.face_enabled` (bool)
- `bot.vision.gesture_enabled` (bool)
- `bot.vision.aruco_enabled` (bool)
- `bot.vision.is_any_detection_active` (bool)
- `bot.vision.is_subscribed` (bool): True if data-topic subscriptions are open.

```python
if bot.vision.yolo_enabled:
    print("YOLO is running on the robot")
```

---

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **PIPELINE MANAGEMENT**

These lower-level methods let you control the vision pipeline lifecycle and per-detector state directly.

##### `vision.start_vision()` / `vision.stop_vision()`

Start or stop the entire robot-side vision pipeline. `start_vision()` also opens data-topic subscriptions automatically.

```python
bot.vision.start_vision()   # Starts pipeline + subscribes to data topics
bot.vision.stop_vision()    # Stops pipeline, resets all detector flags
```

##### `vision.enable_detector(detector)` / `vision.disable_detector(detector)`

Enable or disable a single detector by name. Calls the corresponding `/robot/enable_<detector>` or `/robot/disable_<detector>` ROS 2 service and mirrors the state to `/vision/control`.

**Parameters:**
- `detector` (str): One of `'yolo'`, `'face'`, `'pose'`, `'gesture'`, `'aruco'`

```python
bot.vision.enable_detector('yolo')
bot.vision.disable_detector('face')
```

##### `vision.toggle_detector(detector, enable)`

Convenience wrapper — enable or disable a detector in one call.

```python
bot.vision.toggle_detector('pose', True)   # enable
bot.vision.toggle_detector('pose', False)  # disable
```

##### `vision.subscribe_to_vision_pipeline()` / `vision.unsubscribe_from_vision_pipeline()`

Manually open or close data-topic subscriptions (YOLO detections, face detections, pose landmarks, gestures, ArUco IDs, nearest person). Subscriptions are opened automatically by `start_vision()` and `enable_detection()`.

```python
bot.vision.subscribe_to_vision_pipeline()    # Start receiving data
bot.vision.unsubscribe_from_vision_pipeline() # Stop receiving data, clear buffers
```

---

#### `get_detections(class_filter=None)`

Return the latest detection results from the vision pipeline.

**Parameters:**
- `class_filter` (str | None): If given, return only detections where `class` matches. If `None`, return all detections

**Returns:** `list[dict]` — each dict has the schema:

```python
{
    'class':      str,    # e.g. 'person', 'bottle'
    'confidence': float,  # 0.0–1.0
    'bbox':       [x, y, w, h],  # pixels, top-left origin
    'center_x':   float,  # pixels
    'center_y':   float   # pixels
}
```

```python
# Get all detections
all_dets = bot.get_detections()
for det in all_dets:
    print(f"{det['class']}: {det['confidence']:.2f}")

# Filter by class
people = bot.get_detections(class_filter='person')
print(f"Found {len(people)} people")
```

#### `get_nearest_person()`

Return the nearest person detection based on distance estimation. Available both as `bot.get_nearest_person()` and `bot.vision.get_nearest_person()`.

**Returns:** `dict | None`

```python
person = bot.get_nearest_person()
if person:
    print(f"Nearest person is at ({person['center_x']}, {person['center_y']})")
```

---

#### `wait_for_detection(target_class, timeout=5.0)`

Block until a detection of `target_class` appears, or timeout.

**Parameters:**
- `target_class` (str): The class name to look for (e.g. `'person'`, `'bottle'`, `'cat'`)
- `timeout` (float): Maximum seconds to wait (default: `5.0`)

**Returns:** `list[dict]` — matching detections (same schema as `get_detections()`), or `[]` on timeout

```python
# Wait for a bottle to appear
results = bot.wait_for_detection('bottle', timeout=10.0)
if results:
    print(f"Found {len(results)} bottle(s)")
else:
    print("No bottle detected")
```

---

#### `get_faces()`

Return the latest face detections.

**Returns:** `list[dict]` — each dict has the schema:

```python
{
    'bbox': [x, y, w, h],
    'confidence': float,
    'landmarks': {
        'nose':      [x, y],
        'left_eye':  [x, y],
        'right_eye': [x, y],
        'left_ear':  [x, y],
        'right_ear': [x, y]
    }
}
```

```python
faces = bot.get_faces()
for face in faces:
    print(f"Face confidence: {face['confidence']:.2f}")
    print(f"Nose position: {face['landmarks']['nose']}")
```

---

#### `wait_for_face(timeout=5.0)`

Block until any face is detected, or timeout.

**Parameters:**
- `timeout` (float): Maximum seconds to wait (default: `5.0`)

**Returns:** `list[dict]` — list of face dicts (same schema as `get_faces()`), or `[]` on timeout

```python
faces = bot.wait_for_face(timeout=10.0)
if faces:
    print(f"Face detected! Confidence: {faces[0]['confidence']:.2f}")
```

---

#### `get_pose_keypoints()`

Return the latest pose landmarks (33 MediaPipe body-pose landmarks).

**Returns:** `list` — the raw pose landmarks list as decoded from the `/vision/pose_landmarks` JSON topic.

```python
keypoints = bot.get_pose_keypoints()
if keypoints:
    print(f"Received {len(keypoints)} landmarks")
```

---

#### `wait_for_pose(timeout=5.0)`

Block until pose keypoints are detected, or timeout.

**Parameters:**
- `timeout` (float): Maximum seconds to wait (default: `5.0`)

**Returns:** `list` — the landmarks list (same as `get_pose_keypoints()`), or `[]` on timeout

```python
pose = bot.wait_for_pose(timeout=10.0)
if pose:
    print(f"Detected {len(pose)} landmarks")
```

---

#### `get_gesture()`

Return the current gesture class name, or `None` if no hand is detected.

**Returns:** `str | None`

**Valid gesture names:** `Thumb_Up`, `Thumb_Down`, `Open_Palm`, `Pointing_Up`, `Victory`, `ILoveYou`

```python
gesture = bot.get_gesture()
if gesture:
    print(f"Gesture: {gesture}")
```

---

#### `get_gesture_full()`

Return the full gesture results list including hand landmarks.

**Returns:** `list[dict]` — each dict has the schema:

```python
{
    'gesture':        str,    # e.g. 'Thumb_Up'
    'confidence':     float,  # 0.0–1.0
    'handedness':     str,    # 'Left' or 'Right'
    'hand_landmarks': [       # 21 MediaPipe hand landmarks
        {'name': str, 'x': int, 'y': int},
        ...
    ]
}
```

Returns `[]` if no hand is detected.

```python
results = bot.get_gesture_full()
if results:
    g = results[0]
    print(f"{g['gesture']} ({g['handedness']} hand, {g['confidence']:.2f})")
```

---

#### `wait_for_gesture(gesture_name, timeout=5.0)`

Block until a specific gesture is detected, or timeout. Matching is case-insensitive.

**Parameters:**
- `gesture_name` (str): The gesture name to wait for (e.g. `'Thumb_Up'`)
- `timeout` (float): Maximum seconds to wait (default: `5.0`)

**Returns:** `dict | None` — the full gesture dict (same schema as `get_gesture_full()`), or `None` on timeout

```python
# Wait for a thumbs up
result = bot.wait_for_gesture('Thumb_Up', timeout=10.0)
if result:
    print(f"Thumbs up from {result['handedness']} hand!")
```

---

#### `get_aruco_markers()`

Return the latest detected ArUco marker IDs.

**Returns:** `list[int]` — integer marker IDs currently visible

```python
markers = bot.get_aruco_markers()
for marker_id in markers:
    print(f"Marker #{marker_id} detected")
```

---

#### `wait_for_marker(marker_id, timeout=5.0)`

Block until a specific ArUco marker ID is detected, or timeout.

**Parameters:**
- `marker_id` (int): The integer marker ID to look for
- `timeout` (float): Maximum seconds to wait (default: `5.0`)

**Returns:** `bool` — `True` if the marker was found, `False` on timeout

```python
found = bot.wait_for_marker(marker_id=1, timeout=10.0)
if found:
    print("Marker 1 is visible!")
```

---

### Vision Examples

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **EXAMPLE A: OBJECT DETECTION LOOP**

```python
from bonicbot_bridge import BonicBot
import time

with BonicBot() as bot:
    # Activate camera and enable object detection
    bot.system.start_camera()
    bot.enable_detection('yolo')
    time.sleep(1)  # Allow pipeline to initialize

    # Print detections for 10 seconds
    start = time.time()
    while (time.time() - start) < 10:
        detections = bot.get_detections()
        for det in detections:
            print(f"  {det['class']}: {det['confidence']:.2f} at ({det['center_x']}, {det['center_y']})")
        time.sleep(0.5)

    # Clean up
    bot.disable_detection()
    bot.system.stop_camera()
```

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **EXAMPLE B: FACE-TRIGGERED WAVE**

```python
from bonicbot_bridge import BonicBot
import time

with BonicBot() as bot:
    bot.system.start_camera()
    bot.enable_detection('face')
    time.sleep(1)

    print("Waiting for a face...")
    face = bot.wait_for_face(timeout=10.0)

    if face:
        print(f"Face found! Confidence: {face['confidence']:.2f}")
        # Wave hello
        bot.move_left_arm(90, 30)
        time.sleep(1)
        bot.move_left_arm(0, 0)
    else:
        print("No face detected in time")

    bot.disable_detection()
    bot.system.stop_camera()
```

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **EXAMPLE C: GESTURE-CONTROLLED MOVEMENT**

```python
from bonicbot_bridge import BonicBot
import time

with BonicBot() as bot:
    bot.system.start_camera()
    bot.enable_detection('gesture')
    time.sleep(1)

    print("Gesture control active! Show your hand...")
    print("  Thumb_Up    = forward")
    print("  Thumb_Down  = backward")
    print("  Open_Palm   = stop")

    start = time.time()
    while (time.time() - start) < 30:  # Run for 30 seconds
        gesture = bot.get_gesture()

        if gesture == 'Thumb_Up':
            bot.move_forward(0.3, duration=1.0)
        elif gesture == 'Thumb_Down':
            bot.move_backward(0.3, duration=1.0)
        elif gesture == 'Open_Palm':
            bot.stop()

        time.sleep(0.2)

    bot.stop()
    bot.disable_detection()
    bot.system.stop_camera()
```

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **EXAMPLE D: ARUCO DOCKING**

```python
from bonicbot_bridge import BonicBot

with BonicBot() as bot:
    bot.system.start_camera()
    bot.enable_detection('aruco', dictionary='DICT_4X4_50')

    print("Looking for marker #1...")
    marker = bot.wait_for_marker(marker_id=1, timeout=15.0)

    if marker:
        distance = marker['distance_m']
        center_x = marker['center_x']
        print(f"Marker #1 found: {distance:.2f}m away")
        print(f"Horizontal position: {center_x:.0f}px from left edge")

        # Simple bearing: if center_x < 320, marker is to the left
        if center_x < 280:
            print("Marker is to the LEFT")
        elif center_x > 360:
            print("Marker is to the RIGHT")
        else:
            print("Marker is CENTERED")
    else:
        print("Marker #1 not found")

    bot.disable_detection()
    bot.system.stop_camera()
```

<br/>

<p align="center"> 〰️ 〰️ 〰️ 〰️ 〰️ </p>


> #### 🔹 **EXAMPLE E: POSE-BASED INTERACTION**

```python
from bonicbot_bridge import BonicBot
import time

IMAGE_CENTER_X = 320  # Assuming 640px wide image

with BonicBot() as bot:
    bot.system.start_camera()
    bot.enable_detection('pose')
    time.sleep(1)

    print("Stand in front of the camera — robot will follow your nose!")
    pose = bot.wait_for_pose(timeout=10.0)

    if pose:
        nose = pose['nose']
        nose_x = nose['x']
        print(f"Nose at x={nose_x}")

        if nose_x < IMAGE_CENTER_X - 50:
            print("You're on the LEFT — turning left")
            bot.turn_left(0.3, duration=0.5)
        elif nose_x > IMAGE_CENTER_X + 50:
            print("You're on the RIGHT — turning right")
            bot.turn_right(0.3, duration=0.5)
        else:
            print("You're CENTERED — staying put")
    else:
        print("No pose detected")

    bot.disable_detection()
    bot.system.stop_camera()
```

## 🔧 Advanced Usage

### System Subscriptions

The `bot.system` namespace provides direct access to subscribe to various core ROS 2 topics:

- `bot.system.subscribe_to_map(callback)` — Attach to the live `/map` stream
- `bot.system.subscribe_to_odom(callback, throttle_rate=100)` — Filtered odometry
- `bot.system.subscribe_to_robot_state(callback)` — Robot lifecycle state
- `bot.system.subscribe_to_mapping_active(callback)` — SLAM boolean status
- `bot.system.subscribe_to_navigation_active(callback)` — Navigation boolean status
- `bot.system.subscribe_to_current_goal(callback)` — Active navigation goal
- `bot.system.subscribe_to_locations_list(callback)` — Live JSON list of saved named locations
- `bot.system.subscribe_to_map_available(callback)` — Boolean flag when a saved map is detected on disk

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
from bonicbot_bridge import (
    BonicBot, 
    ConnectionError, 
    NavigationError,
    PreciseMotionError,
    ExploreError,
    ExploreTimeoutError,
    ServoError,
    SensorError,
    VisionError,
    CameraError,
    SystemControlError
)

try:
    with BonicBot(host='192.168.1.100') as bot:
        bot.start_navigation()
        bot.go_to(5, 5)

except ConnectionError as e:
    print(f"Could not connect to robot: {e}")

except NavigationError as e:
    print(f"Navigation failed: {e}")

except Exception as e:
    print(f"An error occurred: {e}")
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

<details><summary>Click to view all topics and services</summary>

The library communicates with these ROS2 topics and services:

**Topics:**

- `/cmd_vel` (geometry_msgs/Twist) - Robot movement commands
- `/diff_cont/odom` (nav_msgs/Odometry) - Robot position feedback (raw)
- `/odometry/filtered` (nav_msgs/Odometry) - Filtered odometry (EKF)
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goals
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - Initial pose for localization
- `/robot/state` (std_msgs/String) - Robot lifecycle state
- `/robot/nav_status` (std_msgs/String) - Navigation status updates
- `/robot/distance_to_goal` (std_msgs/Float32) - Distance feedback
- `/robot/mapping_active` (std_msgs/Bool) - SLAM mapping active flag
- `/robot/navigation_active` (std_msgs/Bool) - Navigation active flag
- `/robot/explore_active` (std_msgs/Bool) - Exploration active flag
- `/joint_states` (sensor_msgs/JointState) - Servo position feedback
- `/left_arm_controller/commands` (std_msgs/Float64MultiArray) - Left arm [shoulder, elbow]
- `/right_arm_controller/commands` (std_msgs/Float64MultiArray) - Right arm [shoulder, elbow]
- `/head_controller/commands` (std_msgs/Float64MultiArray) - Head [yaw]
- `/left_gripper_controller/commands` (std_msgs/Float64MultiArray) - Left gripper [finger1]
- `/right_gripper_controller/commands` (std_msgs/Float64MultiArray) - Right gripper [finger1]
- `/camera/image_raw/compressed` (sensor_msgs/CompressedImage) - Camera images
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Camera metadata
- `/robot/camera_active` (std_msgs/Bool) - Camera status
- `/vision/control` (std_msgs/String) - Vision config JSON (published by SDK → robot)
- `/vision/yolo_detections` (std_msgs/String) - JSON array of YOLO object detections
- `/vision/face_detections` (std_msgs/String) - JSON array of face detections
- `/vision/pose_landmarks` (std_msgs/String) - JSON of 33 MediaPipe body pose landmarks
- `/vision/gestures` (std_msgs/String) - JSON array of hand gesture results
- `/vision/aruco_ids` (std_msgs/String) - JSON array of ArUco marker IDs
- `/vision/nearest_person` (std_msgs/String) - JSON object of nearest person detection
- `/vision/yolo_active` (std_msgs/Bool) - YOLO detector running status
- `/vision/pose_active` (std_msgs/Bool) - Pose detector running status
- `/vision/face_active` (std_msgs/Bool) - Face detector running status
- `/vision/gesture_active` (std_msgs/Bool) - Gesture detector running status
- `/vision/aruco_active` (std_msgs/Bool) - ArUco detector running status
- `/robot/map_available` (std_msgs/Bool) - Saved map existence flag
- `/robot/goto_location` (std_msgs/String) - Named location navigation command
- `/robot/save_location` (std_msgs/String) - Save named location
- `/robot/delete_location` (std_msgs/String) - Delete named location
- `/robot/locations_list` (std_msgs/String) - JSON list of saved location names
- `/robot/current_goal` (geometry_msgs/PoseStamped) - Active navigation goal
- `/explore/status` (ExploreStatus) - explore_lite lifecycle events
- `/explore/resume` (std_msgs/Bool) - Pause/resume exploration
- `/explore/frontiers` (visualization_msgs/MarkerArray) - Frontier visualization
- `/global_costmap/costmap` (nav_msgs/OccupancyGrid) - Nav2 costmap verification
- `/map` (nav_msgs/OccupancyGrid) - SLAM map

**Services:**

- `/robot/start_mapping` (std_srvs/Trigger) - Start SLAM mapping
- `/robot/stop_mapping` (std_srvs/Trigger) - Stop SLAM mapping
- `/robot/save_map` (std_srvs/Trigger) - Save current map
- `/robot/start_navigation` (std_srvs/Trigger) - Start navigation
- `/robot/stop_navigation` (std_srvs/Trigger) - Stop navigation
- `/robot/cancel_navigation` (std_srvs/Trigger) - Cancel current goal
- `/robot/start_camera` (std_srvs/Trigger) - Start camera system
- `/robot/stop_camera` (std_srvs/Trigger) - Stop camera system
- `/robot/start_explore` (std_srvs/Trigger) - Start explore_lite
- `/robot/stop_explore` (std_srvs/Trigger) - Stop explore_lite
- `/robot/start_vision` (std_srvs/Trigger) - Start vision pipeline
- `/robot/stop_vision` (std_srvs/Trigger) - Stop vision pipeline
- `/robot/enable_yolo` (std_srvs/Trigger) - Enable YOLO detector
- `/robot/disable_yolo` (std_srvs/Trigger) - Disable YOLO detector
- `/robot/enable_face` (std_srvs/Trigger) - Enable face detector
- `/robot/disable_face` (std_srvs/Trigger) - Disable face detector
- `/robot/enable_pose` (std_srvs/Trigger) - Enable pose detector
- `/robot/disable_pose` (std_srvs/Trigger) - Disable pose detector
- `/robot/enable_gesture` (std_srvs/Trigger) - Enable gesture detector
- `/robot/disable_gesture` (std_srvs/Trigger) - Disable gesture detector
- `/robot/enable_aruco` (std_srvs/Trigger) - Enable ArUco detector
- `/robot/disable_aruco` (std_srvs/Trigger) - Disable ArUco detector

</details>

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
- Or force-start navigation in SLAM mode: `bot.system.start_navigation(force=True)`

**Import Error**

```
ModuleNotFoundError: No module named 'bonicbot_bridge'
```

- Install the library: `pip install bonicbot-bridge`
- For development: `pip install -e .` from the source directory

**Vision Detection Returns Empty Results**

- Confirm the robot's `vision_pipeline.py` node is running
- Confirm camera is active: `bot.activate_camera_hardware()` must be called first
- Check the active detectors: `print(bot.get_active_mode())` — should return the detector name you enabled
- Ensure the required model files exist on the robot (e.g. `~/models/yolov8n.onnx` for YOLO, `~/models/gesture_recognizer.task` for gestures)

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

