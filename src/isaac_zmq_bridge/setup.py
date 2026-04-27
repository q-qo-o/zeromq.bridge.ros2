from setuptools import find_packages, setup
from glob import glob
import os

package_name = "isaac_zmq_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="macabaka",
    maintainer_email="qdsyqaaa@gmail.com",
    description="Dynamic Isaac Sim ZeroMQ Bridge with auto-discovery and on-demand topic allocation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Isaac Sim Dynamic Bridge System
            "zmq_to_ros2_bridge = isaac_zmq_bridge.dynamic_zmq_ros2_bridge:main",
            "ros2_to_zmq_bridge = isaac_zmq_bridge.dynamic_ros2_zmq_bridge:main",
        ],
    },
)
