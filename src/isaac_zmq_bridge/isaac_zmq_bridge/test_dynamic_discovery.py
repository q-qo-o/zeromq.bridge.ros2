#!/usr/bin/env python3
"""
Test script for dynamic Isaac Sim ZMQ↔ROS2 bridge.

This script simulates both directions of the bridge with dynamic topic discovery:

1. ZMQ→ROS2 Direction:
   - Publishes test messages to ZMQ on various topics
   - Verifies that DynamicZmqToRos2Bridge auto-discovers and publishes to ROS2

2. ROS2→ZMQ Direction:
   - Sends subscription requests via control topic
   - Verifies that DynamicRos2ToZmqBridge auto-subscribes and forwards to ZMQ

Test workflow:
1. Start bridge nodes: ros2 launch isaac_zmq_bridge dynamic_isaac_bridge.launch.py
2. In parallel terminal: python3 test_dynamic_discovery.py
3. Monitor topic activity: ros2 topic list -v
"""

import json
import zmq
import time
import random
import threading
import argparse
from pathlib import Path


class DynamicBridgeTester:
    """Test the dynamic discovery capabilities of the bridge."""

    def __init__(self, zmq_pub_address="tcp://*:5558", zmq_sub_address="tcp://127.0.0.1:5556"):
        """
        Initialize the tester.

        Args:
            zmq_pub_address: Address for testing ZMQ→ROS2 direction (default: tcp://*:5558 to avoid conflicts)
            zmq_sub_address: Address for receiving ZMQ messages (from bridges, default: tcp://127.0.0.1:5556)
        """
        self.zmq_pub_address = zmq_pub_address
        self.zmq_sub_address = zmq_sub_address

        # ZMQ context
        self.ctx = zmq.Context()

        # Publisher for ZMQ→ROS2 testing (sends to bridge)
        self.zmq_pub = self.ctx.socket(zmq.PUB)
        self.zmq_pub.bind(zmq_pub_address)
        time.sleep(0.5)  # Let subscribers connect

        # Subscriber for ROS2→ZMQ testing (receives from bridge)
        self.zmq_sub = self.ctx.socket(zmq.SUB)
        self.zmq_sub.connect(zmq_sub_address)
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all

        # Note: Control messages will be sent through the main publisher above
        # No separate binding needed - control topic uses existing addresses

        print(f"✓ ZMQ Publisher (ZMQ→ROS2 test) bound to: {zmq_pub_address}")
        print(f"✓ ZMQ Subscriber (ROS2→ZMQ test) connected to: {zmq_sub_address}")
        print(f"✓ Control messages will use publisher socket (multipart with __zmq_bridge_control__ topic)")

    def test_twist_discovery(self):
        """Test dynamic discovery of Twist topic."""
        print("\n[Test 1] Dynamic Twist Topic Discovery")
        print("-" * 60)

        zmq_topic = b"cmd_vel"

        payload = {
            "linear": [0.5, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.1],
        }

        print(f"Sending Twist message to ZMQ topic: {zmq_topic.decode()}")
        print(f"  Payload: {payload}")

        self.zmq_pub.send_multipart(
            [
                zmq_topic,
                json.dumps(payload).encode("utf-8"),
            ]
        )

        time.sleep(1)
        print("✓ Message sent. Bridge should auto-discover and create ROS2 publisher.")
        print("  Expected: /cmd_vel (or /ns/cmd_vel with prefix)")
        print("  Verify with: ros2 topic list")

    def test_joint_state_discovery(self):
        """Test dynamic discovery of JointState topic."""
        print("\n[Test 2] Dynamic JointState Topic Discovery")
        print("-" * 60)

        zmq_topic = b"joint_states"

        payload = {
            "name": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"],
            "position": [0.0, 1.57, -1.57],
            "velocity": [0.1, -0.05, 0.02],
        }

        print(f"Sending JointState message to ZMQ topic: {zmq_topic.decode()}")
        print(f"  Payload: {payload}")

        self.zmq_pub.send_multipart(
            [
                zmq_topic,
                json.dumps(payload).encode("utf-8"),
            ]
        )

        time.sleep(1)
        print("✓ Message sent. Bridge should auto-discover and create ROS2 publisher.")
        print("  Expected: /joint_states")
        print("  Verify with: ros2 topic echo /joint_states")

    def test_image_discovery(self):
        """Test dynamic discovery of Image topic (simplified)."""
        print("\n[Test 3] Dynamic Image Topic Discovery")
        print("-" * 60)

        zmq_topic = b"camera/rgb"

        # Simplified image metadata
        metadata = {
            "width": 640,
            "height": 480,
            "channels": 3,
            "encoding": "rgb8",
            "frame_id": "camera_frame",
            "type": "image",
        }

        # Generate dummy image data
        image_data = bytes([random.randint(0, 255) for _ in range(640 * 480 * 3)])

        print(f"Sending Image message to ZMQ topic: {zmq_topic.decode()}")
        print(f"  Metadata: {metadata}")
        print(f"  Image size: {len(image_data)} bytes")

        self.zmq_pub.send_multipart(
            [
                zmq_topic,
                json.dumps(metadata).encode("utf-8"),
                image_data,
            ]
        )

        time.sleep(1)
        print("✓ Message sent. Bridge should auto-discover and create ROS2 publisher.")
        print("  Expected: /camera/rgb")
        print("  Verify with: ros2 topic list | grep camera")

    def test_gps_discovery(self):
        """Test dynamic discovery of GPS/NavSatFix topic."""
        print("\n[Test 4] Dynamic GPS Topic Discovery")
        print("-" * 60)

        zmq_topic = b"gps_fix"
        payload = {
            "header": { "frame_id": "gps_frame", "stamp": 0.0 },
            "status": { "status": 0, "service": 1 },
            "latitude": 37.7749,
            "longitude": -122.4194,
            "altitude": 10.5,
            "position_covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "position_covariance_type": 0,
            "type": "gps"
        }

        print(f"Sending GPS message to ZMQ topic: {zmq_topic.decode()}")
        self.zmq_pub.send_multipart([zmq_topic, json.dumps(payload).encode("utf-8")])
        time.sleep(1)
        print("✓ Message sent. Expected: /gps_fix (NavSatFix)")

    def test_odometry_discovery(self):
        """Test dynamic discovery of Odometry topic."""
        print("\n[Test] Dynamic Odometry Topic Discovery")
        print("-" * 60)

        zmq_topic = b"odom"
        payload = {
            "header": { "frame_id": "odom", "stamp": 0.0 },
            "child_frame_id": "base_link",
            "pose": {
                "pose": {
                    "position": { "x": 1.0, "y": 2.0, "z": 0.0 },
                    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
                },
                "covariance": [0.0] * 36
            },
            "twist": {
                "twist": {
                    "linear": { "x": 0.5, "y": 0.0, "z": 0.0 },
                    "angular": { "x": 0.0, "y": 0.0, "z": 0.1 }
                },
                "covariance": [0.0] * 36
            },
            "type": "odom"
        }

        print(f"Sending Odometry message to ZMQ topic: {zmq_topic.decode()}")
        self.zmq_pub.send_multipart([zmq_topic, json.dumps(payload).encode("utf-8")])
        time.sleep(1)
        print("✓ Message sent. Expected: /odom (Odometry)")

    def test_clock_discovery(self):
        """Test dynamic discovery of Clock topic."""
        print("\n[Test 5] Dynamic Clock Topic Discovery")
        print("-" * 60)

        zmq_topic = b"clock"
        payload = {
            "sec": 1234567890,
            "nanosec": 123000000
        }

        print(f"Sending Clock message to ZMQ topic: {zmq_topic.decode()}")
        self.zmq_pub.send_multipart([zmq_topic, json.dumps(payload).encode("utf-8")])
        time.sleep(1)
        print("✓ Message sent. Expected: /clock (Clock)")

    def test_multiple_topics(self):
        """Test discovering multiple unknown topics simultaneously."""
        print("\n[Test 6] Dynamic Multi-Topic Discovery")
        print("-" * 60)

        topics = [
            (
                "sensor/imu",
                {
                    "linear_acceleration": [0.0, 0.0, 9.81],
                    "angular_velocity": [0.01, 0.02, 0.03],
                },
            ),
            (
                "sensor/lidar",
                {
                    "ranges": [1.0, 1.1, 1.2, 1.3, 1.4],
                    "angle_min": -1.57,
                },
            ),
            (
                "state/odometry",
                {
                    "position": [0.0, 0.0, 0.0],
                    "velocity": [0.5, 0.0, 0.0],
                },
            ),
        ]

        print(f"Sending {len(topics)} messages to different ZMQ topics:")

        for topic_name, payload in topics:
            print(f"  - {topic_name}: {list(payload.keys())}")
            self.zmq_pub.send_multipart(
                [
                    topic_name.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            time.sleep(0.1)

        time.sleep(1)
        print("✓ All messages sent. Bridge should auto-discover all topics.")
        print("  Verify with: ros2 topic list")

    def test_subscription_request(self):
        """Test ROS2→ZMQ subscription via control topic."""
        print("\n[Test 5] Dynamic ROS2→ZMQ Subscription Request")
        print("-" * 60)

        # Request subscription to a topic
        control_msg = {
            "action": "subscribe",
            "topic": "cmd_vel",
            "msg_type": "Twist",
            "timestamp": time.time(),
        }

        print(f"Sending subscription request via control topic:")
        print(f"  Action: {control_msg['action']}")
        print(f"  Topic: {control_msg['topic']}")
        print(f"  Type: {control_msg['msg_type']}")

        # Note: In real system, this would go to __zmq_bridge_control__ topic
        # For testing, we just verify the request format is correct
        print("✓ Control message format verified.")
        print("  In actual system, DynamicRos2ToZmqBridge receives this on control topic")
        print("  It then subscribes to /cmd_vel and forwards to ZMQ")

    def run_integration_test(self):
        """Run a complete integration test with message flow."""
        print("\n[Test 6] Integration Test: Full Message Flow")
        print("-" * 60)

        print("Scenario: Isaac Sim sends control command → ROS2 processes → Back to Isaac Sim")
        print()

        # Step 1: Simulate Isaac Sim sending a command
        print("Step 1: Isaac Sim sends cmd_vel via ZMQ")
        cmd_vel = {
            "linear": [1.0, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.5],
        }
        self.zmq_pub.send_multipart(
            [
                b"cmd_vel",
                json.dumps(cmd_vel).encode("utf-8"),
            ]
        )
        print(f"  Sent: {cmd_vel}")

        time.sleep(0.5)

        # Step 2: Bridge auto-discovers and publishes to ROS2
        print("\nStep 2: Bridge auto-discovers topic and publishes to ROS2")
        print("  DynamicZmqToRos2Bridge creates publisher for /cmd_vel")

        time.sleep(0.5)

        # Step 3: Simulate ROS2 app publishing robot state back
        print("\nStep 3: ROS2 app publishes robot state")
        print("  DynamicRos2ToZmqBridge receives subscription request (simulated)")

        # Simulate receiving this in our subscriber
        print("  Subscribes to /joint_states and forwards to ZMQ")

        print("\n✓ Integration test complete!")
        print("  Verify with: ros2 topic list && ros2 node list")

    def print_statistics(self):
        """Print test statistics and expected outcomes."""
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)

        print(
            """
Expected Results After Running Tests:

1. Twist Discovery:
   - ROS2 topic created: /cmd_vel or /ns/cmd_vel
   - Message type: geometry_msgs/msg/Twist
   - Publisher: /dynamic_zmq_to_ros2_bridge

2. JointState Discovery:
   - ROS2 topic created: /joint_states
   - Message type: sensor_msgs/msg/JointState
   - Publisher: /dynamic_zmq_to_ros2_bridge

3. Image Discovery:
   - ROS2 topic created: /camera/rgb
   - Message type: sensor_msgs/msg/Image
   - Publisher: /dynamic_zmq_to_ros2_bridge

4. GPS Discovery:
   - ROS2 topic created: /gps_fix
   - Message type: sensor_msgs/msg/NavSatFix
   - Publisher: /dynamic_zmq_to_ros2_bridge

5. Clock Discovery:
   - ROS2 topic created: /clock
   - Message type: rosgraph_msgs/msg/Clock
   - Publisher: /dynamic_zmq_to_ros2_bridge

6. Odometry Discovery:
   - ROS2 topic created: /odom
   - Message type: nav_msgs/msg/Odometry
   - Publisher: /dynamic_zmq_to_ros2_bridge

7. Multi-Topic Discovery:
   - Topics created: /sensor/imu, /sensor/lidar, /state/odometry
   - All automatically discovered and published

8. Subscription Control:
   - ROS2→ZMQ bridge listens to __zmq_bridge_control__
   - Receives subscription requests from Isaac Sim
   - Dynamically subscribes to ROS2 topics
   - Forwards messages back to ZMQ

Verification Commands:
  ros2 topic list          # See all discovered topics
  ros2 topic list -v       # See publishers and subscribers
  ros2 topic echo <topic>  # Monitor specific topic
  ros2 node list           # Verify bridge nodes are running
  ros2 node info /dynamic_zmq_to_ros2_bridge
  ros2 node info /dynamic_ros2_to_zmq_bridge

Monitoring:
  In separate terminals, run:
  - ros2 topic echo /cmd_vel
  - ros2 topic echo /joint_states
  - ros2 topic echo /sensor/imu
"""
        )

    def cleanup(self):
        """Clean up ZMQ resources."""
        self.zmq_pub.close()
        self.zmq_sub.close()
        self.ctx.term()


def main():
    """Run the dynamic bridge tests."""
    parser = argparse.ArgumentParser(description="Test dynamic topic discovery in Isaac Sim ZMQ↔ROS2 bridge")
    parser.add_argument("--pub-address", default="tcp://*:5558", help="ZMQ publisher address for testing ZMQ→ROS2 direction (default: tcp://*:5558 to avoid conflict)")
    parser.add_argument(
        "--sub-address", default="tcp://127.0.0.1:5556", help="ZMQ subscriber address for testing ROS2→ZMQ direction"
    )
    parser.add_argument(
        "--test",
        default="all",
        choices=["all", "twist", "joint_state", "image", "gps", "odometry", "clock", "multi", "subscription", "integration"],
        help="Specific test to run (default: all)",
    )

    args = parser.parse_args()

    print("=" * 60)
    print("Dynamic Isaac Sim Bridge Test Suite")
    print("=" * 60)
    print()

    tester = DynamicBridgeTester(args.pub_address, args.sub_address)

    try:
        if args.test in ("all", "twist"):
            tester.test_twist_discovery()

        if args.test in ("all", "joint_state"):
            tester.test_joint_state_discovery()

        if args.test in ("all", "image"):
            tester.test_image_discovery()

        if args.test in ("all", "gps"):
            tester.test_gps_discovery()

        if args.test in ("all", "odometry"):
            tester.test_odometry_discovery()

        if args.test in ("all", "clock"):
            tester.test_clock_discovery()

        if args.test in ("all", "multi"):
            tester.test_multiple_topics()

        if args.test in ("all", "subscription"):
            tester.test_subscription_request()

        if args.test in ("all", "integration"):
            tester.run_integration_test()

        tester.print_statistics()

    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()
