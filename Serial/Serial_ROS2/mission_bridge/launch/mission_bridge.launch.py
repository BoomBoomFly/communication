"""mission_bridge 节点 launch 文件：从 config/mission_bridge.yaml 加载参数并启动桥接节点。

参数文件加载方式参照 Serial/serial_driver_ros/launch/serial_driver.launch.py。
临时覆盖参数可追加 ROS 参数，例如：
    ros2 launch mission_bridge mission_bridge.launch.py --ros-args -p serial.port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mission_bridge'),
        'config',
        'mission_bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='mission_bridge',
            executable='mission_bridge_node',
            name='mission_bridge',
            output='screen',
            parameters=[config_path])
    ])
