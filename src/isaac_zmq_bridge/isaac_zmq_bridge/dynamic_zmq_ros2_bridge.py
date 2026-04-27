"""
Dynamic ZMQ to ROS2 Bridge Node

This module implements a unified bridge node that dynamically discovers and creates
ROS2 publishers based on received ZMQ messages. No pre-configuration needed.

Architecture:
- Subscribe to all ZMQ topics using wildcard
- Inspect each message to determine type
- Automatically create ROS2 publisher if not exists
- Route messages to appropriate handlers
"""

import json
import zmq
import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image, Imu, NavSatFix
from std_msgs.msg import Float32, Float64, Int32, Float32MultiArray, Float64MultiArray, Int32MultiArray
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry


class DynamicZmqToRos2Bridge(Node):
    """
    Dynamic bridge that automatically discovers topics from ZMQ messages
    and creates corresponding ROS2 publishers.

    No pre-configuration needed - topics are discovered as messages arrive.
    """

    def __init__(self):
        super().__init__("dynamic_zmq_to_ros2_bridge")
        self.bridge = CvBridge()

        # Declare parameters
        # isaac_zmq_pub_address: Port where Isaac Sim publishes data (PUB). Bridge connects here to subscribe.
        self.declare_parameter("isaac_zmq_pub_address", "tcp://127.0.0.1:25556")
        self.declare_parameter("topic_prefix", "")  # Optional prefix for ROS2 topics

        # Get parameters
        self.isaac_zmq_pub_address = self.get_parameter("isaac_zmq_pub_address").value
        self.topic_prefix = self.get_parameter("topic_prefix").value

        # Dynamic topic publishers - topic_name -> {publisher, type, frame_count}
        self.ros2_publishers = {}

        # Message type hints cache - learned from received messages
        self.topic_types = {}

        # Initialize single ZMQ subscriber with wildcard subscription
        ctx = zmq.Context.instance()
        self.zmq_sub = ctx.socket(zmq.SUB)
        self.zmq_sub.connect(self.isaac_zmq_pub_address)
        # Subscribe to ALL topics using empty string (wildcard)
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, b"")

        self.get_logger().info(
            f"Dynamic ZMQ→ROS2 Bridge started\n"
            f"  Bridge SUB (Isaac PUB) Address: {self.isaac_zmq_pub_address}\n"
            f"  Topic Prefix: {self.topic_prefix or '(none)'}\n"
            f"  Mode: Auto-discovery (wildcard subscription)"
        )

        # Statistics
        self.msg_counts = {}
        self.discovered_topics = set()

        # Timer for polling ZMQ
        self.timer = self.create_timer(0.001, self._tick)  # 1ms polling

        # Stats timer
        self.stats_timer = self.create_timer(10.0, self._log_stats)

    def _infer_message_type(self, payload_str):
        """
        Infer message type from JSON payload content.

        Returns: 'twist', 'joint_state', 'clock', or None
        """
        try:
            data = json.loads(payload_str)

            # Check for Twist (has 'linear' and 'angular')
            if "linear" in data and "angular" in data:
                return "twist"

            # Check for JointState (has 'name' and 'position')
            if "name" in data and "position" in data:
                return "joint_state"

            # Check for Clock (has 'sec' and 'nanosec')
            if "sec" in data and "nanosec" in data:
                return "clock"

            # Check for Image (has 'type' == 'image')
            if data.get("type") == "image":
                return "image"

            # Check for IMU (has 'type' == 'imu')
            if data.get("type") == "imu":
                return "imu"
                
            # Check for GPS (has 'type' == 'gps')
            if data.get("type") == "gps" or "latitude" in data:
                return "navsatfix"

            # Check for Odometry (has 'type' == 'odom')
            if data.get("type") == "odom" or ("pose" in data and "twist" in data and "child_frame_id" in data):
                return "odometry"
            
            # Check for Primitives (has 'data')
            if "data" in data:
                val = data["data"]
                if isinstance(val, int):
                    return "int32"
                elif isinstance(val, float):
                    return "float64"  # Default to double for safety
                elif isinstance(val, list):
                    if not val:
                        return "float64_array" # Empty list default
                    if isinstance(val[0], int):
                        return "int32_array"
                    return "float64_array" # Default float array
                
            return None
        except:
            return None

    def _ros2_topic_name(self, zmq_topic):
        """
        Convert ZMQ topic to a valid ROS2 topic name.

        Returns None if the ZMQ topic contains invalid characters for ROS2
        (such as addresses like tcp://...).
        """
        # Cleanup leading slashes
        clean_name = zmq_topic.lstrip("/")

        # Check for invalid characters (ROS2 only allows alphanum and underscores)
        # Any string containing : or / (after lstrip) is likely an address or complex path
        if "://" in clean_name or ":" in clean_name:
            return None

        # Ensure it's not empty
        if not clean_name:
            return None

        # Apply prefix if needed
        if self.topic_prefix:
            prefix = self.topic_prefix.strip("/")
            return f"/{prefix}/{clean_name}"
        else:
            return f"/{clean_name}"

    def _get_or_create_publisher(self, zmq_topic, msg_type):
        """Get existing publisher or create new one."""
        if zmq_topic in self.ros2_publishers:
            return self.ros2_publishers[zmq_topic]["publisher"]

        ros_topic = self._ros2_topic_name(zmq_topic)

        # Skip if topic name is invalid for ROS2
        if ros_topic is None:
            self.get_logger().debug(f"Skipping ZMQ topic (invalid for ROS2): {zmq_topic}")
            return None

        # Create appropriate publisher based on type
        try:
            if msg_type == "twist":
                pub = self.create_publisher(Twist, ros_topic, 10)
            elif msg_type == "joint_state":
                pub = self.create_publisher(JointState, ros_topic, 10)
            elif msg_type == "int32_array":
                pub = self.create_publisher(Int32MultiArray, ros_topic, 10)
            elif msg_type == "float64_array":
                pub = self.create_publisher(Float64MultiArray, ros_topic, 10)
            elif msg_type == "float32_array":
                pub = self.create_publisher(Float32MultiArray, ros_topic, 10)
            elif msg_type == "int32":
                pub = self.create_publisher(Int32, ros_topic, 10)
            elif msg_type == "float64":
                pub = self.create_publisher(Float64, ros_topic, 10)
            elif msg_type == "float32":
                pub = self.create_publisher(Float32, ros_topic, 10)
            elif msg_type == "clock":
                pub = self.create_publisher(Clock, ros_topic, 10)
            elif msg_type == "image":
                # Use best-effort QoS for images
                qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
                pub = self.create_publisher(Image, ros_topic, qos)
            elif msg_type == "imu":
                pub = self.create_publisher(Imu, ros_topic, 10)
            elif msg_type == "navsatfix":
                pub = self.create_publisher(NavSatFix, ros_topic, 10)
            elif msg_type == "odometry":
                pub = self.create_publisher(Odometry, ros_topic, 10)
            else:
                self.get_logger().warn(f"Unknown message type: {msg_type}")
                return None
        except Exception as exc:
            self.get_logger().warn(f"Failed to create ROS2 publisher for {ros_topic}: {exc}")
            return None

        # Cache publisher info
        self.ros2_publishers[zmq_topic] = {"publisher": pub, "type": msg_type, "ros_topic": ros_topic, "frame_count": 0}

        self.discovered_topics.add(zmq_topic)
        self.msg_counts[zmq_topic] = 0

        self.get_logger().info(f"[Discovery] New topic: {zmq_topic} → {ros_topic} " f"({msg_type})")

        return pub

    def _tick(self):
        """Poll ZMQ and route messages to appropriate ROS2 publishers."""
        try:
            while self.zmq_sub.poll(0):
                # Receive multipart message
                parts = self.zmq_sub.recv_multipart(flags=zmq.NOBLOCK)

                if len(parts) < 2:
                    self.get_logger().warn("Received incomplete message")
                    continue

                # Parse topic name
                zmq_topic = parts[0].decode("utf-8")

                # Infer message type from payload
                msg_type = self._infer_message_type(parts[1].decode("utf-8"))

                if msg_type is None:
                    self.get_logger().debug(f"Could not infer type for topic: {zmq_topic}")
                    continue

                # Get or create publisher
                publisher = self._get_or_create_publisher(zmq_topic, msg_type)

                if publisher is None:
                    continue

                # Handle message
                self._handle_message(zmq_topic, msg_type, parts)

                # Update statistics
                self.msg_counts[zmq_topic] = self.msg_counts.get(zmq_topic, 0) + 1

        except zmq.Again:
            pass
        except Exception as exc:
            self.get_logger().error(f"Error in message handling: {exc}", throttle_duration_sec=1.0)

    def _handle_message(self, zmq_topic, msg_type, parts):
        """Handle a message based on its type."""
        pub_info = self.ros2_publishers.get(zmq_topic)
        if not pub_info:
            return

        publisher = pub_info["publisher"]

        try:
            if msg_type == "twist":
                self._handle_twist(publisher, parts)
            elif msg_type == "joint_state":
                self._handle_joint_state(publisher, parts)
            elif msg_type == "clock":
                self._handle_clock(publisher, parts)
            elif msg_type == "int32_array":
                self._handle_int32_array(publisher, parts)
            elif msg_type == "float64_array":
                self._handle_float64_array(publisher, parts)
            elif msg_type == "float32_array":
                self._handle_float32_array(publisher, parts)
            elif msg_type == "int32":
                self._handle_int32(publisher, parts)
            elif msg_type == "float64":
                self._handle_float64(publisher, parts)
            elif msg_type == "float32":
                self._handle_float32(publisher, parts)
            elif msg_type == "image":
                self._handle_image(publisher, parts)
            elif msg_type == "imu":
                self._handle_imu(publisher, parts)
            elif msg_type == "navsatfix":
                self._handle_navsatfix(publisher, parts)
            elif msg_type == "odometry":
                self._handle_odometry(publisher, parts)
        except Exception as exc:
            self.get_logger().warn(f"Failed to handle {msg_type} message on {zmq_topic}: {exc}", throttle_duration_sec=5.0)

    def _handle_twist(self, publisher, parts):
        """Handle Twist message."""
        data = json.loads(parts[1].decode("utf-8"))

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = data.get("linear", [0.0, 0.0, 0.0])
        msg.angular.x, msg.angular.y, msg.angular.z = data.get("angular", [0.0, 0.0, 0.0])
        publisher.publish(msg)

    def _handle_joint_state(self, publisher, parts):
        """Handle JointState message."""
        data = json.loads(parts[1].decode("utf-8"))

        msg = JointState()
        
        timestamp = data.get("timestamp")
        if timestamp is not None:
            msg.header.stamp.sec = int(timestamp)
            msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            
        msg.name = data.get("name", [])
        msg.position = data.get("position", [])
        msg.velocity = data.get("velocity", [])
        msg.effort = data.get("effort", [])
        publisher.publish(msg)

    def _handle_clock(self, publisher, parts):
        """Handle Clock message."""
        try:
            data = json.loads(parts[1].decode("utf-8"))

            msg = Clock()
            # Ensure types are correct for ROS2 msg (int, int)
            msg.clock.sec = int(data.get("sec", 0))
            msg.clock.nanosec = int(data.get("nanosec", 0))
            publisher.publish(msg)
        except Exception as exc:
             self.get_logger().warn(f"Failed to handle Clock message: {exc}", throttle_duration_sec=5.0)

    def _handle_image(self, publisher, parts):
        """Handle Image message. Decodes PNG from ZMQ and publishes as raw Image."""
        if len(parts) < 3:
            self.get_logger().warn("Incomplete image message")
            return

        try:
            metadata = json.loads(parts[1].decode("utf-8"))
            image_bytes = parts[2]
            frame_id = metadata.get("frame_id", "camera")

            # Isaac Sim sends PNG encoded image bytes. We need to decode them 
            # to publish as a standard sensor_msgs/Image (raw).
            nparr = np.frombuffer(image_bytes, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)

            if img is None:
                self.get_logger().warn(f"Failed to decode image message from ZMQ")
                return

            height, width = img.shape[:2]
            channels = img.shape[2] if len(img.shape) > 2 else 1

            # Standard Isaac Sim Camera output is BGRA (4 channels)
            encoding = "bgra8" if channels == 4 else ("bgr8" if channels == 3 else "mono8")

            # Use CvBridge for conversion (idiomatic and often more optimized)
            msg = self.bridge.cv2_to_imgmsg(img, encoding=encoding)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id

            publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to handle image message: {exc}", throttle_duration_sec=5.0)

    def _handle_imu(self, publisher, parts):
        """Handle IMU message."""
        data = json.loads(parts[1].decode("utf-8"))

        msg = Imu()
        # Header
        header_data = data.get("header", {})
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = header_data.get("frame_id", "imu_frame")

        # Orientation
        orientation = data.get("orientation", {})
        msg.orientation.x = float(orientation.get("x", 0.0))
        msg.orientation.y = float(orientation.get("y", 0.0))
        msg.orientation.z = float(orientation.get("z", 0.0))
        msg.orientation.w = float(orientation.get("w", 1.0))

        # Angular Velocity
        ang_vel = data.get("angular_velocity", {})
        msg.angular_velocity.x = float(ang_vel.get("x", 0.0))
        msg.angular_velocity.y = float(ang_vel.get("y", 0.0))
        msg.angular_velocity.z = float(ang_vel.get("z", 0.0))

        # Linear Acceleration
        lin_accel = data.get("linear_acceleration", {})
        msg.linear_acceleration.x = float(lin_accel.get("x", 0.0))
        msg.linear_acceleration.y = float(lin_accel.get("y", 0.0))
        msg.linear_acceleration.z = float(lin_accel.get("z", 0.0))

        publisher.publish(msg)

    def _handle_int32(self, publisher, parts):
        """Handle Int32 message."""
        data = json.loads(parts[1].decode("utf-8"))
        msg = Int32()
        msg.data = int(data.get("data", 0))
        publisher.publish(msg)

    def _handle_float64(self, publisher, parts):
        """Handle Float64 message."""
        data = json.loads(parts[1].decode("utf-8"))
        msg = Float64()
        msg.data = float(data.get("data", 0.0))
        publisher.publish(msg)

    def _handle_int32_array(self, publisher, parts):
        """Handle Int32MultiArray message."""
        data = json.loads(parts[1].decode("utf-8"))
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data.get("data", [])]
        publisher.publish(msg)

    def _handle_float64_array(self, publisher, parts):
        """Handle Float64MultiArray message."""
        data = json.loads(parts[1].decode("utf-8"))
        msg = Float64MultiArray()
        msg.data = data.get("data", [])
        publisher.publish(msg)

    def _handle_float32_array(self, publisher, parts):
        """Handle Float32MultiArray message."""
        data = json.loads(parts[1].decode("utf-8"))
        msg = Float32MultiArray()
        msg.data = data.get("data", [])
        publisher.publish(msg)

    def _handle_float32(self, publisher, parts):
        """Handle Float32 message."""
        data = json.loads(parts[1].decode("utf-8"))
        msg = Float32()
        msg.data = float(data.get("data", 0.0))
        publisher.publish(msg)

    def _handle_navsatfix(self, publisher, parts):
        """Handle NavSatFix/GPS message."""
        data = json.loads(parts[1].decode("utf-8"))
        
        msg = NavSatFix()
        # Header
        header_data = data.get("header", {})
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = header_data.get("frame_id", "gps_frame")

        # Status
        status_data = data.get("status", {})
        msg.status.status = int(status_data.get("status", 0))
        msg.status.service = int(status_data.get("service", 1))

        # NavSatFix Data
        msg.latitude = float(data.get("latitude", 0.0))
        msg.longitude = float(data.get("longitude", 0.0))
        msg.altitude = float(data.get("altitude", 0.0))
        
        # Position Covariance (9 elements)
        cov = data.get("position_covariance", [0.0]*9)
        if len(cov) == 9:
            msg.position_covariance = [float(x) for x in cov]
            
        msg.position_covariance_type = int(data.get("position_covariance_type", 0))
        
        publisher.publish(msg)

    def _handle_odometry(self, publisher, parts):
        """Handle Odometry message."""
        data = json.loads(parts[1].decode("utf-8"))
        
        msg = Odometry()
        
        # Header
        header_data = data.get("header", {})
        msg.header.stamp = self.get_clock().now().to_msg() # Or use exact stamp from msg if preferred, but bridge usually timestamps now
        msg.header.frame_id = str(header_data.get("frame_id", "odom"))
        
        msg.child_frame_id = str(data.get("child_frame_id", "base_link"))
        
        # Pose
        pose_obj = data.get("pose", {})
        pose_data = pose_obj.get("pose", {})
        pos = pose_data.get("position", {})
        msg.pose.pose.position.x = float(pos.get("x", 0.0))
        msg.pose.pose.position.y = float(pos.get("y", 0.0))
        msg.pose.pose.position.z = float(pos.get("z", 0.0))
        
        ori = pose_data.get("orientation", {})
        msg.pose.pose.orientation.x = float(ori.get("x", 0.0))
        msg.pose.pose.orientation.y = float(ori.get("y", 0.0))
        msg.pose.pose.orientation.z = float(ori.get("z", 0.0))
        msg.pose.pose.orientation.w = float(ori.get("w", 1.0))
        
        pose_cov = pose_obj.get("covariance", [0.0]*36)
        if len(pose_cov) == 36:
            msg.pose.covariance = [float(x) for x in pose_cov]
            
        # Twist
        twist_obj = data.get("twist", {})
        twist_data = twist_obj.get("twist", {})
        lin = twist_data.get("linear", {})
        msg.twist.twist.linear.x = float(lin.get("x", 0.0))
        msg.twist.twist.linear.y = float(lin.get("y", 0.0))
        msg.twist.twist.linear.z = float(lin.get("z", 0.0))
        
        ang = twist_data.get("angular", {})
        msg.twist.twist.angular.x = float(ang.get("x", 0.0))
        msg.twist.twist.angular.y = float(ang.get("y", 0.0))
        msg.twist.twist.angular.z = float(ang.get("z", 0.0))
        
        twist_cov = twist_obj.get("covariance", [0.0]*36)
        if len(twist_cov) == 36:
            msg.twist.covariance = [float(x) for x in twist_cov]

        publisher.publish(msg)

    def _log_stats(self):
        """Log statistics."""
        total = sum(self.msg_counts.values())
        if total > 0:
            stats_lines = [
                f"Dynamic Topic Discovery (last 10s):",
                f"  Discovered topics: {len(self.discovered_topics)}",
                f"  Total messages: {total}",
            ]

            for topic, count in sorted(self.msg_counts.items()):
                if count > 0:
                    msg_type = self.ros2_publishers[topic]["type"]
                    stats_lines.append(f"  {topic:30} → {msg_type:12} ({count:5} msgs)")

            self.get_logger().info("\n".join(stats_lines))
            # Reset counts
            self.msg_counts = {topic: 0 for topic in self.msg_counts}


def main(args=None):
    """Entry point for dynamic ZMQ to ROS2 bridge."""
    rclpy.init(args=args)
    node = DynamicZmqToRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
