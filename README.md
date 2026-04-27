# ZeroMQ ROS 2 Bridge for Isaac Sim

A dynamic, zero-configuration bridge connecting NVIDIA Isaac Sim and ROS 2 via ZeroMQ. This package eliminates the need for predefined topic lists by dynamically discovering publishers and subscribers at runtime.

## Features

- **Zero-Configuration:** Topics and message types are discovered automatically.
- **Bidirectional Communication:**
  - **Isaac Sim → ROS 2:** Automatically detects incoming ZMQ messages from Isaac Sim, infers their types, and creates corresponding ROS 2 publishers.
  - **ROS 2 → Isaac Sim:** Listens to a ZMQ control topic (`__zmq_bridge_control__`) for subscription requests from Isaac Sim, and dynamically creates ROS 2 subscribers to forward data back to the simulation.
- **Robust Networking:** Handles "Late Joiner" ZMQ issues using periodic subscription heartbeats and uses monotonic network timestamps to prevent data freezing when the simulation timeline is paused.
- **Supported Message Types:**
  - `geometry_msgs/Twist`
  - `sensor_msgs/JointState`
  - `sensor_msgs/Image`
  - `sensor_msgs/Imu`
  - `sensor_msgs/NavSatFix`
  - `nav_msgs/Odometry`
  - `rosgraph_msgs/Clock`
  - Standard primitives and arrays (`Float32`, `Float64`, `Int32`, `Float32MultiArray`, etc.)

## Prerequisites

- ROS 2 (e.g., Humble, Jazzy)
- Python dependencies: `pyzmq`, `opencv-python`, `numpy`, `cv_bridge`
- NVIDIA Isaac Sim (with the corresponding `zeromq.bridge` extension)

## Installation

Ensure your workspace is set up and build the package:

```bash
# Navigate to your workspace
cd ~/WorkSpace/ROS2/IsaacSim

# Build the bridge package
colcon build --packages-select isaac_zmq_bridge

# Source the overlay
source install/setup.bash
```

## Usage

Launch both the ZMQ → ROS 2 and ROS 2 → ZMQ bridge nodes simultaneously using the provided launch file:

```bash
ros2 launch isaac_zmq_bridge isaac_bridge.launch.py
```

### Launch Arguments

You can customize the network ports and ROS 2 topic prefix if necessary:

- `isaac_zmq_pub_address` (default: `tcp://127.0.0.1:25556`): Address where Isaac Sim publishes data.
- `isaac_zmq_sub_address` (default: `tcp://*:25557`): Address where the Bridge publishes data for Isaac Sim to receive.
- `topic_prefix` (default: `""`): Optional prefix to append to all automatically discovered ROS 2 topics.

Example overriding default parameters:
```bash
ros2 launch isaac_zmq_bridge isaac_bridge.launch.py isaac_zmq_pub_address:="tcp://192.168.1.100:25556" topic_prefix:="robot_1"
```

## Architecture

This package consists of two primary nodes:

1. **`dynamic_zmq_ros2_bridge`:** Subscribes to all topics using a ZMQ wildcard (`""`). Upon receiving a message, it parses the JSON payload, infers the message type based on keys (e.g., checking for `linear` and `angular` indicates a `Twist`), and instantiates a ROS 2 publisher on-the-fly.
2. **`dynamic_ros2_zmq_bridge`:** Subscribes to a specific ZMQ control topic (`__zmq_bridge_control__`). When Isaac Sim nodes request data (e.g., a `ZmqSubscribeTwist` node in Omnigraph), they publish an `action: subscribe` request here. This bridge node parses the request, dynamically creates a ROS 2 subscriber for the requested topic, and forwards any received ROS 2 messages as JSON back over ZMQ.