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

**Important**: Camera operations have two parts:
1. **Hardware control** (server-side): Activates/deactivates physical camera
2. **Streaming control** (client-side): Subscribes/unsubscribes to camera images

**Recommended workflow:**

```python
# 1. Activate camera hardware
bot.camera.start_camera_service()

# 2. Start receiving images
bot.start_camera()
bot.camera.wait_for_image(timeout=3.0)

# 3. Use camera
bot.save_image("photo.jpg")

# 4. Stop receiving images
bot.stop_camera()

# 5. Deactivate hardware (important for performance!)
bot.camera.stop_camera_service()
```

---

#### Hardware Control (Server-Side)

##### `camera.start_camera_service()`

Activate the robot's physical camera hardware.

```python
bot.camera.start_camera_service()  # Turn ON camera
```

##### `camera.stop_camera_service()`

Deactivate camera hardware to free up resources.

```python
bot.camera.stop_camera_service()  # Turn OFF camera
```

##### `system.is_camera_active()`

Check if camera hardware is currently activated.

```python
is_active = bot.system.is_camera_active()  # Returns True/False
```

---

#### Streaming Control (Client-Side)

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

#### Image Access

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

##### `camera.wait_for_image(timeout)`

Wait for first image to arrive.

```python
bot.camera.wait_for_image(timeout=5.0)  # Wait up to 5 seconds
```

##### `camera.get_camera_info()`

Get camera metadata (resolution, distortion model, etc.).

```python
info = bot.camera.get_camera_info()
print(f"Resolution: {info['width']}x{info['height']}")
```

---

#### Complete Example

```python
with BonicBot(host='192.168.1.100') as bot:
    # Activate hardware
    bot.camera.start_camera_service()
    
    # Start receiving images
    bot.start_camera()
    bot.camera.wait_for_image(timeout=3.0)
    
    # Capture photo
    bot.save_image("destination.jpg")
    
    # Stop receiving
    bot.stop_camera()
    
    # Deactivate hardware
    bot.camera.stop_camera_service()
```

### Servo Control Methods

**Architecture**: The servo system uses separate ROS2 controller topics for each group (left arm, right arm, head, grippers).

---

#### `move_left_arm(shoulder, elbow)` / `move_right_arm(shoulder, elbow)`

Move robot arms to specified angles.

**Parameters:**
- `shoulder` (float): Shoulder pitch angle (-45° to 180°)
- `elbow` (float): Elbow angle (0° to 50°)

```python
bot.move_left_arm(90, 30)   # Left arm up
bot.move_right_arm(45, 20)  # Right arm halfway
```

---

#### Gripper Control

##### `set_grippers(left, right)`

Control both grippers simultaneously.

**Parameters:**
- `left` (float): Left gripper angle (-45° to 60°)
- `right` (float): Right gripper angle (-45° to 60°)

```python
bot.set_grippers(30, 30)    # Partial open
```

##### `set_left_gripper(angle)` / `set_right_gripper(angle)`

Control individual grippers independently.

**Parameters:**
- `angle` (float): Gripper angle (-45° to 60°)

```python
bot.servo.set_left_gripper(30)   # Left gripper only
bot.servo.set_right_gripper(45)  # Right gripper only
```

##### `open_grippers()` / `close_grippers()`

Convenience methods for both grippers.

```python
bot.open_grippers()         # Open both to 60°
bot.close_grippers()        # Close both to 0°
```

---

#### Neck Control

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

---

#### Technical Details

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

### Vision & Detection Methods

> **ℹ️ Architecture:** All vision inference runs on the robot's onboard `vision_pipeline.py` ROS 2 node. The bridge SDK simply sends configuration commands and receives detection results — no local models or GPU required on your machine.

---

#### Detection Modes

The vision pipeline uses string literals to define all available pipeline modes.

| String | What it detects |
|--------|----------------|
| `'yolo'` | YOLO object detection (80 COCO classes: person, bottle, chair, etc.) |
| `'face'` | Human faces with 5 facial landmarks |
| `'pose'` | 17 COCO body keypoints |
| `'gesture'` | Hand gestures (Thumb_Up, Open_Palm, Victory, etc.) |
| `'aruco'` | ArUco fiducial markers with full pose (tvec, rvec, distance) |

---

#### `enable_detection(mode, model='yolov8n', dictionary=None)`

Enable a vision detection pipeline mode on the remote robot.

This publishes a JSON configuration to `/vision/control` which toggles the requested detector on the robot's `vision_pipeline.py` node. Only one mode is activated per call (all others are set to `False`).

**Parameters:**
- `mode` (str): The detection mode to enable (`'yolo'`, `'face'`, `'pose'`, `'gesture'`, or `'aruco'`)
- `model` (str): YOLO model name (default: `'yolov8n'`). Reserved for future multi-model support
- `dictionary` (str | None): ArUco dictionary name. Reserved for future use

**Supported ArUco dictionaries:** `DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_5X5_50`, `DICT_6X6_50`

**Returns:** `dict` response from the service

**Raises:**
- `BonicBotError` if `mode` is not a valid detection mode
- `VisionError` if the publish call fails

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

#### Status Properties

Check if specific detectors are currently active on the robot using properties on `bot.vision`:

- `bot.vision.vision_active` (bool): True if the vision pipeline is running.
- `bot.vision.yolo_enabled` (bool)
- `bot.vision.pose_enabled` (bool)
- `bot.vision.face_enabled` (bool)
- `bot.vision.gesture_enabled` (bool)
- `bot.vision.aruco_enabled` (bool)
- `bot.vision.is_any_detection_active` (bool)

```python
if bot.vision.yolo_enabled:
    print("YOLO is running on the robot")
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

Return the nearest person detection based on distance estimation.

**Returns:** `dict | None`

```python
person = bot.vision.get_nearest_person()
if person:
    print(f"Nearest person is at ({person['center_x']}, {person['center_y']})")
```

---

#### `wait_for_detection(target_class, timeout=5.0)`

Block until a detection of `target_class` appears, or timeout.

**Parameters:**
- `target_class` (str): The class name to look for (e.g. `'person'`, `'bottle'`, `'cat'`)
- `timeout` (float): Maximum seconds to wait (default: `5.0`)

**Returns:** `dict | None` — the first matching detection dict, or `None` on timeout

```python
# Wait for a bottle to appear
bottle = bot.wait_for_detection('bottle', timeout=10.0)
if bottle:
    print(f"Bottle found at ({bottle['center_x']}, {bottle['center_y']})")
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

**Returns:** `dict | None` — the first face dict, or `None` on timeout

```python
face = bot.wait_for_face(timeout=10.0)
if face:
    print(f"Face detected! Confidence: {face['confidence']:.2f}")
```

---

#### `get_pose_keypoints()`

Return the latest pose keypoints (17 COCO keypoints).

**Returns:** `dict` — each key is a keypoint name, each value has the schema:

```python
{
    'nose':           {'x': int, 'y': int, 'confidence': float},
    'left_shoulder':  {'x': int, 'y': int, 'confidence': float},
    'right_shoulder': {'x': int, 'y': int, 'confidence': float},
    'left_elbow':     {'x': int, 'y': int, 'confidence': float},
    'right_elbow':    {'x': int, 'y': int, 'confidence': float},
    # ... 17 COCO keypoints total, absolute pixel coordinates
}
```

```python
keypoints = bot.get_pose_keypoints()
if keypoints:
    nose = keypoints['nose']
    print(f"Nose at ({nose['x']}, {nose['y']}), conf: {nose['confidence']:.2f}")
```

---

#### `wait_for_pose(timeout=5.0)`

Block until pose keypoints are detected, or timeout.

**Parameters:**
- `timeout` (float): Maximum seconds to wait (default: `5.0`)

**Returns:** `dict | None` — the full keypoints dict, or `None` on timeout

```python
pose = bot.wait_for_pose(timeout=10.0)
if pose:
    print(f"Detected {len(pose)} keypoints")
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

Return the full gesture result including hand landmarks.

**Returns:** `dict` with the schema:

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

Returns `{}` if no hand is detected.

```python
result = bot.get_gesture_full()
if result:
    print(f"{result['gesture']} ({result['handedness']} hand, {result['confidence']:.2f})")
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

Return the latest ArUco marker detections.

**Returns:** `list[dict]` — each dict has the schema:

```python
{
    'id':         int,
    'corners':    [[x, y], [x, y], [x, y], [x, y]],  # 4 corner points
    'center_x':   float,
    'center_y':   float,
    'calibrated': bool,    # False = approximate intrinsics, distance is estimated
    'tvec':       [tx, ty, tz],  # translation in meters, Z-axis points forward
    'rvec':       [rx, ry, rz],  # rotation vector (Rodrigues)
    'distance_m': float          # distance to marker in meters
}
```

> **Note:** When `calibrated` is `False`, approximate camera intrinsics were used — the `distance_m` and `tvec` values are estimates. The `tvec` Z-axis points forward from the marker.

```python
markers = bot.get_aruco_markers()
for m in markers:
    print(f"Marker #{m['id']}: {m['distance_m']:.2f}m away")
```

---

#### `wait_for_marker(marker_id, timeout=5.0)`

Block until a specific ArUco marker ID is detected, or timeout.

**Parameters:**
- `marker_id` (int): The integer marker ID to look for
- `timeout` (float): Maximum seconds to wait (default: `5.0`)

**Returns:** `dict | None` — the matching marker dict, or `None` on timeout

```python
marker = bot.wait_for_marker(marker_id=1, timeout=10.0)
if marker:
    print(f"Marker 1 found at {marker['distance_m']:.2f}m")
```

---

### Vision Examples

#### Example A: Object Detection Loop

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

#### Example B: Face-Triggered Wave

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

#### Example C: Gesture-Controlled Movement

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

#### Example D: ArUco Docking

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

#### Example E: Pose-Based Interaction

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
- `/vision/control` (std_msgs/String) - Vision config JSON (published by SDK → robot)
- `/vision/yolo_detections` (std_msgs/String) - JSON array of YOLO object detections
- `/vision/face_detections` (std_msgs/String) - JSON array of face detections
- `/vision/pose_landmarks` (std_msgs/String) - JSON dict of 33 MediaPipe body pose landmarks
- `/vision/gestures` (std_msgs/String) - JSON dict of hand gesture result
- `/vision/aruco_ids` (std_msgs/String) - JSON array of ArUco marker IDs and poses
- `/vision/nearest_person` (std_msgs/String) - JSON object of nearest person detection
- `/vision/yolo_active` (std_msgs/Bool) - YOLO detector running status
- `/vision/pose_active` (std_msgs/Bool) - Pose detector running status
- `/vision/face_active` (std_msgs/Bool) - Face detector running status
- `/vision/gesture_active` (std_msgs/Bool) - Gesture detector running status
- `/vision/aruco_active` (std_msgs/Bool) - ArUco detector running status

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

**Vision Detection Returns Empty Results**

- Confirm the robot's `vision_pipeline.py` node is running
- Confirm camera is streaming: `bot.camera.start_camera_service()` must be called first
- Check the active detectors: `print(bot.get_active_mode())` — the detector you enabled should show `True`
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

