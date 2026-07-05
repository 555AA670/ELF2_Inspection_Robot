from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = Path(__file__).with_name("robot_localization_ekf.yaml")
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_imu",
            arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "base_footprint", "--child-frame-id", "imu_link"]
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[str(config_path)],
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])
