"""
Launch file for fully dynamic Isaac Sim ZMQ↔ROS2 bridge.

No pre-configuration needed - topics are discovered and allocated at runtime.

Usage:
    ros2 launch isaac_zmq_bridge isaac_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for dynamic bridges."""
    
    # Declare launch arguments
    # isaac_zmq_pub_address: Port where Isaac Sim publishes (PUB).
    isaac_zmq_pub_address_arg = DeclareLaunchArgument(
        "isaac_zmq_pub_address",
        default_value="tcp://127.0.0.1:25556",
        description="Address where Isaac Sim publishes data (PUB port)"
    )
    
    # isaac_zmq_sub_address: Port where Isaac Sim listens (SUB).
    isaac_zmq_sub_address_arg = DeclareLaunchArgument(
        "isaac_zmq_sub_address",
        default_value="tcp://*:25557",
        description="Address where Isaac Sim listens for data (SUB port)"
    )
    
    topic_prefix_arg = DeclareLaunchArgument(
        "topic_prefix",
        default_value="",
        description="Optional prefix for ROS2 topics"
    )
    
    # Get configuration values
    isaac_zmq_pub_address = LaunchConfiguration("isaac_zmq_pub_address")
    isaac_zmq_sub_address = LaunchConfiguration("isaac_zmq_sub_address")
    topic_prefix = LaunchConfiguration("topic_prefix")
    
    # ZMQ to ROS2 bridge node (auto-discovery)
    zmq_to_ros2_node = Node(
        package="isaac_zmq_bridge",
        executable="zmq_to_ros2_bridge",
        name="zmq_to_ros2_bridge",
        output="screen",
        parameters=[
            {
                "isaac_zmq_pub_address": isaac_zmq_pub_address,
                "topic_prefix": topic_prefix,
            }
        ],
    )
    
    # ROS2 to ZMQ bridge node (subscription request handling)
    ros2_to_zmq_node = Node(
        package="isaac_zmq_bridge",
        executable="ros2_to_zmq_bridge",
        name="ros2_to_zmq_bridge",
        output="screen",
        parameters=[
            {
                "isaac_zmq_sub_address": isaac_zmq_sub_address,
                "isaac_zmq_pub_address": isaac_zmq_pub_address,
            }
        ],
    )
    
    return LaunchDescription([
        # Declare arguments
        isaac_zmq_pub_address_arg,
        isaac_zmq_sub_address_arg,
        topic_prefix_arg,
        
        # Launch nodes
        zmq_to_ros2_node,
        ros2_to_zmq_node,
    ])
