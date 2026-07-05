import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 nav2_bringup 的路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_path = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        # 1. 启动 Nav2 核心组件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'map': '/root/my_new_map.yaml',
                'params_file': '/root/nav2_ackermann_params.yaml'
            }.items()
        ),
        
        # 2. 将 Nav2 默认的 /cmd_vel 转发到我们小车的 /cmd_vel_raw 以便过避障节点
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            arguments=['/cmd_vel', '/cmd_vel_raw'],
            output='screen'
        ),

    ])
