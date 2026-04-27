"""
Dynamic ROS2 to ZMQ Bridge Node

This module implements a unified bridge node that dynamically subscribes to ROS2 topics
based on subscription requests from Isaac Sim via the control topic protocol.

Architecture:
- Listen to control topic: __zmq_bridge_control__
- Parse subscription/unsubscription requests
- Dynamically create/destroy ROS2 subscribers
- Forward messages to ZMQ publisher
"""

import json
import zmq
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image, Imu, NavSatFix
from std_msgs.msg import Float32, Float64, Int32, Float32MultiArray, Float64MultiArray, Int32MultiArray
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

class DynamicRos2ToZmqBridge(Node):
    """
    Dynamic bridge that subscribes to ROS2 topics based on subscription requests
    from Isaac Sim via the control topic protocol.

    Subscriptions are managed dynamically - no pre-configuration needed.
    """

    def __init__(self):
        super().__init__("dynamic_ros2_to_zmq_bridge")

        # Declare parameters
        # isaac_zmq_sub_address: The port Isaac Sim listens on (SUB). Bridge binds here to publish.
        # isaac_zmq_pub_address: The port Isaac Sim publishes on (PUB). Bridge connects here to subscribe.
        self.declare_parameter("isaac_zmq_sub_address", "tcp://*:25557")
        self.declare_parameter("isaac_zmq_pub_address", "tcp://127.0.0.1:25556")

        # Get parameters
        self.isaac_zmq_sub_address = self.get_parameter("isaac_zmq_sub_address").value
        self.isaac_zmq_pub_address = self.get_parameter("isaac_zmq_pub_address").value

        # Dynamic ROS2 subscribers - ros_topic -> {subscription, zmq_topic, type}
        self.ros2_subscribers = {}

        # Initialize single ZMQ publisher with unified address
        ctx = zmq.Context.instance()
        self.zmq_pub = ctx.socket(zmq.PUB)
        # Set linger to 0 for quick release on restart
        self.zmq_pub.setsockopt(zmq.LINGER, 0)

        try:
            # Isaac Sim connects SUB socket, so we must bind
            self.zmq_pub.bind(self.isaac_zmq_sub_address)
        except zmq.error.ZMQError as exc:
            if exc.errno == zmq.EADDRINUSE:
                self.get_logger().error(
                    f"Port {self.isaac_zmq_sub_address} is already in use. "
                    "Another bridge might be running, or Isaac Sim is binding on this port instead of connecting. "
                    "Try to kill other bridge processes or change the port."
                )
            raise exc

        # Initialize control topic subscriber
        # Listens to Isaac Sim's PUB port for control commands
        self.control_sub = ctx.socket(zmq.SUB)
        self.control_sub.connect(self.isaac_zmq_pub_address)
        self.control_sub.setsockopt(zmq.SUBSCRIBE, b"__zmq_bridge_control__")

        self.get_logger().info(
            f"Dynamic ROS2→ZMQ Bridge started\n"
            f"  Bridge PUB (Isaac SUB) Address: {self.isaac_zmq_sub_address}\n"
            f"  Bridge Control SUB (Isaac PUB) Address: {self.isaac_zmq_pub_address}\n"
            f"  Mode: Subscription request handling"
        )

        # Statistics
        self.msg_counts = {}
        self.topic_map = {}  # ros_topic -> zmq_topic mapping

        # Timer for polling control messages and checking subscriptions
        self.timer = self.create_timer(0.01, self._tick)

        # Stats timer
        self.stats_timer = self.create_timer(10.0, self._log_stats)

    def _handle_control_message(self, control_msg):
        """
        Process subscription/unsubscription requests from control topic.

        Expected format:
        {
            "action": "subscribe" | "unsubscribe",
            "topic": "ros_topic_name",
            "msg_type": "Twist" | "JointState" | "Image" | ...,
            "timestamp": unix_timestamp
        }
        """
        try:
            data = json.loads(control_msg.decode("utf-8"))

            action = data.get("action")
            ros_topic = data.get("topic")
            msg_type = data.get("msg_type")

            if not ros_topic:
                self.get_logger().warn("Control message missing 'topic' field")
                return

            if action == "subscribe":
                self._subscribe_to_topic(ros_topic, msg_type)
            elif action == "unsubscribe":
                self._unsubscribe_from_topic(ros_topic)
            else:
                self.get_logger().warn(f"Unknown control action: {action}")

        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse control message: {e}")
        except Exception as e:
            self.get_logger().error(f"Error handling control message: {e}")

    def _subscribe_to_topic(self, ros_topic, msg_type="generic"):
        """
        Subscribe to a ROS2 topic and forward messages to ZMQ.

        Args:
            ros_topic: ROS2 topic name (e.g., "cmd_vel", "joint_states")
            msg_type: Message type hint (e.g., "Twist", "JointState")
        """
        # Normalize topic name (remove leading slash)
        ros_topic_normalized = ros_topic.lstrip("/")

        if ros_topic_normalized in self.ros2_subscribers:
            self.get_logger().debug(f"Already subscribed to: {ros_topic_normalized}")
            return

        # Determine ZMQ topic name (same as ROS2 topic by default)
        zmq_topic = ros_topic_normalized

        # Create subscription based on message type
        if msg_type == "Twist":
            sub = self.create_subscription(Twist, ros_topic, lambda msg: self._on_twist(msg, zmq_topic), 10)
        elif msg_type == "JointState":
            sub = self.create_subscription(JointState, ros_topic, lambda msg: self._on_joint_state(msg, zmq_topic), 10)
        elif msg_type == "Image":
            sub = self.create_subscription(Image, ros_topic, lambda msg: self._on_image(msg, zmq_topic), 10)
        elif msg_type == "IMU" or msg_type == "Imu":
            sub = self.create_subscription(Imu, ros_topic, lambda msg: self._on_imu(msg, zmq_topic), 10)
        elif msg_type == "Float32":
            sub = self.create_subscription(Float32, ros_topic, lambda msg: self._on_float32(msg, zmq_topic), 10)
        elif msg_type == "Float64":
            sub = self.create_subscription(Float64, ros_topic, lambda msg: self._on_float64(msg, zmq_topic), 10)
        elif msg_type == "Int32":
            sub = self.create_subscription(Int32, ros_topic, lambda msg: self._on_int32(msg, zmq_topic), 10)
        elif msg_type == "Float32MultiArray":
            sub = self.create_subscription(Float32MultiArray, ros_topic, lambda msg: self._on_float32_array(msg, zmq_topic), 10)
        elif msg_type == "Float64MultiArray":
            sub = self.create_subscription(Float64MultiArray, ros_topic, lambda msg: self._on_float64_array(msg, zmq_topic), 10)
        elif msg_type == "Int32MultiArray":
            sub = self.create_subscription(Int32MultiArray, ros_topic, lambda msg: self._on_int32_array(msg, zmq_topic), 10)
        elif msg_type == "NavSatFix" or msg_type == "gps":
            sub = self.create_subscription(NavSatFix, ros_topic, lambda msg: self._on_navsatfix(msg, zmq_topic), 10)
        elif msg_type == "Clock":
            sub = self.create_subscription(Clock, ros_topic, lambda msg: self._on_clock(msg, zmq_topic), 10)
        elif msg_type == "Odometry" or msg_type == "odom":
            sub = self.create_subscription(Odometry, ros_topic, lambda msg: self._on_odometry(msg, zmq_topic), 10)
        else:
            # Generic case - try to handle common types
            self.get_logger().warn(f"Unknown message type: {msg_type}, skipping subscription to {ros_topic}")
            return

        # Cache subscription info
        self.ros2_subscribers[ros_topic_normalized] = {
            "subscription": sub,
            "zmq_topic": zmq_topic,
            "type": msg_type,
            "ros_topic": ros_topic,
        }

        self.msg_counts[zmq_topic] = 0
        self.topic_map[ros_topic_normalized] = zmq_topic

        self.get_logger().info(f"[Subscribe] {ros_topic} → {zmq_topic} " f"({msg_type})")

    def _unsubscribe_from_topic(self, ros_topic):
        """
        Unsubscribe from a ROS2 topic.

        Args:
            ros_topic: ROS2 topic name
        """
        # Normalize topic name
        ros_topic_normalized = ros_topic.lstrip("/")

        if ros_topic_normalized not in self.ros2_subscribers:
            self.get_logger().debug(f"Not subscribed to: {ros_topic_normalized}")
            return

        sub_info = self.ros2_subscribers[ros_topic_normalized]
        zmq_topic = sub_info["zmq_topic"]

        # Cleanly destroy subscription
        if "subscription" in sub_info:
            self.destroy_subscription(sub_info["subscription"])

        del self.ros2_subscribers[ros_topic_normalized]
        if zmq_topic in self.msg_counts:
            del self.msg_counts[zmq_topic]
        if ros_topic_normalized in self.topic_map:
            del self.topic_map[ros_topic_normalized]

        self.get_logger().info(f"[Unsubscribe] {ros_topic_normalized}")

    def _on_twist(self, msg: Twist, zmq_topic: str):
        """Handle Twist message from ROS2."""
        try:
            payload = {
                "linear": [msg.linear.x, msg.linear.y, msg.linear.z],
                "angular": [msg.angular.x, msg.angular.y, msg.angular.z],
            }
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Twist to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_joint_state(self, msg: JointState, zmq_topic: str):
        """Handle JointState message from ROS2."""
        try:
            payload = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
                "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish JointState to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_image(self, msg: Image, zmq_topic: str):
        """Handle Image message from ROS2 efficiently."""
        try:
            # Determine channels correctly
            channels = 1
            if "8" in msg.encoding:
                if "bgra" in msg.encoding.lower() or "rgba" in msg.encoding.lower():
                    channels = 4
                elif "bgr" in msg.encoding.lower() or "rgb" in msg.encoding.lower():
                    channels = 3

            metadata = {
                "width": msg.width,
                "height": msg.height,
                "channels": channels,
                "encoding": msg.encoding,
                "frame_id": msg.header.frame_id,
                "type": "image",
            }

            # Use memoryview or msg.data directly to avoid expensive bytes() copy
            # msg.data in rclpy is usually a buffer or array.array (depends on version)
            data_to_send = msg.data
            if not isinstance(data_to_send, (bytes, bytearray, memoryview)):
                # Fallback only if strictly necessary
                data_to_send = memoryview(msg.data)

            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(metadata).encode("utf-8"),
                    data_to_send,
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Image to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_imu(self, msg: Imu, zmq_topic: str):
        """Handle IMU message from ROS2."""
        try:
            payload = {
                "header": {
                    "frame_id": msg.header.frame_id,
                    "stamp": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
                },
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w,
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z,
                },
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z,
                },
                "type": "imu",
            }
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish IMU to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_float32(self, msg: Float32, zmq_topic: str):
        """Handle Float32 message from ROS2."""
        try:
            payload = {"data": float(msg.data)}
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Float32 to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_float64(self, msg: Float64, zmq_topic: str):
        """Handle Float64 message from ROS2."""
        try:
            payload = {"data": float(msg.data)}
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Float64 to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_int32(self, msg: Int32, zmq_topic: str):
        """Handle Int32 message from ROS2."""
        try:
            payload = {"data": int(msg.data)}
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Int32 to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_float32_array(self, msg: Float32MultiArray, zmq_topic: str):
        """Handle Float32MultiArray message from ROS2."""
        try:
            payload = {"data": list(msg.data)}
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Float32MultiArray to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_float64_array(self, msg: Float64MultiArray, zmq_topic: str):
        """Handle Float64MultiArray message from ROS2."""
        try:
            payload = {"data": list(msg.data)}
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Float64MultiArray to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_int32_array(self, msg: Int32MultiArray, zmq_topic: str):
        """Handle Int32MultiArray message from ROS2."""
        try:
            payload = {"data": list(msg.data)}
            self.zmq_pub.send_multipart(
                [
                    zmq_topic.encode("utf-8"),
                    json.dumps(payload).encode("utf-8"),
                ]
            )
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Int32 to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_navsatfix(self, msg: NavSatFix, zmq_topic: str):
        """Handle NavSatFix message from ROS2."""
        try:
            payload = {
                "header": {
                    "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                    "frame_id": msg.header.frame_id
                },
                "status": {
                    "status": msg.status.status,
                    "service": msg.status.service
                },
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude,
                "position_covariance": list(msg.position_covariance),
                "position_covariance_type": msg.position_covariance_type,
                "type": "gps"
            }
            self.zmq_pub.send_multipart([
                zmq_topic.encode("utf-8"),
                json.dumps(payload).encode("utf-8")
            ])
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish NavSatFix to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_clock(self, msg: Clock, zmq_topic: str):
        """Handle Clock message from ROS2."""
        try:
            payload = {
                "sec": msg.clock.sec,
                "nanosec": msg.clock.nanosec
            }
            self.zmq_pub.send_multipart([
                zmq_topic.encode("utf-8"),
                json.dumps(payload).encode("utf-8")
            ])
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Clock to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _on_odometry(self, msg: Odometry, zmq_topic: str):
        """Handle Odometry message from ROS2."""
        try:
            payload = {
                "header": {
                    "frame_id": msg.header.frame_id,
                    "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                },
                "child_frame_id": msg.child_frame_id,
                "pose": {
                    "pose": {
                        "position": {
                            "x": msg.pose.pose.position.x,
                            "y": msg.pose.pose.position.y,
                            "z": msg.pose.pose.position.z
                        },
                        "orientation": {
                            "x": msg.pose.pose.orientation.x,
                            "y": msg.pose.pose.orientation.y,
                            "z": msg.pose.pose.orientation.z,
                            "w": msg.pose.pose.orientation.w
                        }
                    },
                    "covariance": list(msg.pose.covariance)
                },
                "twist": {
                    "twist": {
                        "linear": {
                            "x": msg.twist.twist.linear.x,
                            "y": msg.twist.twist.linear.y,
                            "z": msg.twist.twist.linear.z
                        },
                        "angular": {
                            "x": msg.twist.twist.angular.x,
                            "y": msg.twist.twist.angular.y,
                            "z": msg.twist.twist.angular.z
                        }
                    },
                    "covariance": list(msg.twist.covariance)
                },
                "type": "odom"
            }
            self.zmq_pub.send_multipart([
                zmq_topic.encode("utf-8"),
                json.dumps(payload).encode("utf-8")
            ])
            self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish Odometry to ZMQ: {exc}", throttle_duration_sec=5.0)

    def _tick(self):
        """Poll control messages and handle subscriptions."""
        # Check for control messages
        try:
            while self.control_sub.poll(0):
                parts = self.control_sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    self._handle_control_message(parts[1])
        except zmq.Again:
            pass
        except Exception as exc:
            self.get_logger().error(f"Error polling control messages: {exc}", throttle_duration_sec=1.0)

    def _log_stats(self):
        """Log statistics."""
        total = sum(self.msg_counts.values())

        if len(self.ros2_subscribers) > 0 or total > 0:
            stats_lines = [
                f"Dynamic Subscription Management (last 10s):",
                f"  Active subscriptions: {len(self.ros2_subscribers)}",
            ]

            if total > 0:
                stats_lines.append(f"  Total messages: {total}")
                for zmq_topic, count in sorted(self.msg_counts.items()):
                    if count > 0:
                        stats_lines.append(f"  {zmq_topic:30} ({count:5} msgs)")
            else:
                if len(self.ros2_subscribers) > 0:
                    stats_lines.append("  Waiting for messages...")
                    for ros_topic, sub_info in sorted(self.ros2_subscribers.items()):
                        zmq_topic = sub_info["zmq_topic"]
                        msg_type = sub_info["type"]
                        stats_lines.append(f"  {ros_topic:30} → {zmq_topic:30} ({msg_type})")

            self.get_logger().info("\n".join(stats_lines))
            # Reset counts
            self.msg_counts = {topic: 0 for topic in self.msg_counts}


def main(args=None):
    """Entry point for dynamic ROS2 to ZMQ bridge."""
    rclpy.init(args=args)
    node = DynamicRos2ToZmqBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
