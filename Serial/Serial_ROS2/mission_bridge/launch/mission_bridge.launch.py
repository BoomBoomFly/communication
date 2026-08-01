"""mission_bridge 节点 launch 文件：从 config/mission_bridge.yaml 加载参数并启动桥接节点。

参数文件加载方式参照 Serial/serial_driver_ros/launch/serial_driver.launch.py。
临时覆盖参数可追加 ROS 参数，例如：
    ros2 launch mission_bridge mission_bridge.launch.py --ros-args -p serial.port:=/dev/ttyUSB1
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_bridge(context):
    config_path = os.path.join(
        get_package_share_directory("mission_bridge"),
        "config",
        "mission_bridge.yaml",
    )
    with open(config_path, "r") as config_file:
        document = yaml.safe_load(config_file) or {}
    try:
        parameters = dict(document["mission_bridge"]["ros__parameters"])
    except (KeyError, TypeError):
        raise RuntimeError("mission_bridge.yaml must contain mission_bridge.ros__parameters")
    # Foxy applies a YAML file after an adjacent dictionary in some nested
    # launch paths.  Merge before Node construction so the requested epoch
    # cannot silently fall back to the YAML auto-generation value.
    parameters["protocol.source_epoch"] = int(
        LaunchConfiguration("source_epoch").perform(context))
    return [Node(
        package="mission_bridge",
        executable="mission_bridge_node",
        name="mission_bridge",
        output="screen",
        parameters=[parameters],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "source_epoch", default_value="0",
            description=(
                "Nonzero uint32 epoch for /mission/start/context; 0 generates "
                "a new epoch for standalone bridge use.")),
        OpaqueFunction(function=_start_bridge),
    ])
