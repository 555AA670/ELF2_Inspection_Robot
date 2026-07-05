from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = Path(__file__).resolve().parent
    bridge_script = here / "stm32_ros2_bridge.py"
    avoid_script = here / "laser_avoid_test_node.py"
    ekf_launch = here / "robot_localization_launch.py"
    sllidar_launch = (
        Path(get_package_share_directory("sllidar_ros2"))
        / "launch"
        / "sllidar_c1_launch.py"
    )

    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    wheel_base = LaunchConfiguration("wheel_base")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_raw_topic = LaunchConfiguration("cmd_raw_topic")
    cmd_safe_topic = LaunchConfiguration("cmd_safe_topic")
    action_topic = LaunchConfiguration("action_topic")
    heading_offset_topic = LaunchConfiguration("heading_offset_topic")
    front_stop_distance = LaunchConfiguration("front_stop_distance")
    front_clear_distance = LaunchConfiguration("front_clear_distance")
    lateral_stop_distance = LaunchConfiguration("lateral_stop_distance")
    avoid_release_distance = LaunchConfiguration("avoid_release_distance")
    emergency_stop_distance = LaunchConfiguration("emergency_stop_distance")
    avoid_speed = LaunchConfiguration("avoid_speed")
    avoid_speed_min = LaunchConfiguration("avoid_speed_min")
    avoid_turn = LaunchConfiguration("avoid_turn")
    avoid_turn_kp = LaunchConfiguration("avoid_turn_kp")
    ftg_bubble_radius_m = LaunchConfiguration("ftg_bubble_radius_m")
    ftg_gap_min_distance = LaunchConfiguration("ftg_gap_min_distance")
    ftg_max_range = LaunchConfiguration("ftg_max_range")
    ftg_turn_bias = LaunchConfiguration("ftg_turn_bias")
    return_speed = LaunchConfiguration("return_speed")
    return_turn = LaunchConfiguration("return_turn")
    return_cross_track_kp = LaunchConfiguration("return_cross_track_kp")
    return_yaw_kp = LaunchConfiguration("return_yaw_kp")
    scan_yaw_offset_deg = LaunchConfiguration("scan_yaw_offset_deg")
    angular_sign = LaunchConfiguration("angular_sign")
    junction_stop_enabled = LaunchConfiguration("junction_stop_enabled")
    junction_front_open_distance = LaunchConfiguration("junction_front_open_distance")
    junction_side_open_distance = LaunchConfiguration("junction_side_open_distance")
    junction_side_min_distance = LaunchConfiguration("junction_side_min_distance")
    junction_open_ratio_threshold = LaunchConfiguration("junction_open_ratio_threshold")
    junction_required_open_paths = LaunchConfiguration("junction_required_open_paths")
    junction_confirm_cycles = LaunchConfiguration("junction_confirm_cycles")
    junction_side_start_deg = LaunchConfiguration("junction_side_start_deg")
    junction_side_end_deg = LaunchConfiguration("junction_side_end_deg")

    return LaunchDescription(
        [
            SetEnvironmentVariable('RCUTILS_CONSOLE_THRESHOLD', 'WARN'),
            SetEnvironmentVariable('ROS_LOG_LEVEL', 'WARN'),
            DeclareLaunchArgument("port", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("wheel_base", default_value="0.23"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("cmd_raw_topic", default_value="/cmd_vel_raw"),
            DeclareLaunchArgument("cmd_safe_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("action_topic", default_value="/stm32/action_cmd"),
            DeclareLaunchArgument("heading_offset_topic", default_value="/heading_offset_cmd"),
            DeclareLaunchArgument("front_stop_distance", default_value="0.30"),
            DeclareLaunchArgument("front_clear_distance", default_value="0.45"),
            DeclareLaunchArgument("lateral_stop_distance", default_value="0.25"),
            DeclareLaunchArgument("avoid_release_distance", default_value="0.35"),
            DeclareLaunchArgument("emergency_stop_distance", default_value="0.1"),
            DeclareLaunchArgument("avoid_speed", default_value="0.50"),
            DeclareLaunchArgument("avoid_speed_min", default_value="0.10"),
            DeclareLaunchArgument("avoid_turn", default_value="1.20"),
            DeclareLaunchArgument("avoid_turn_kp", default_value="1.80"),
            DeclareLaunchArgument("ftg_bubble_radius_m", default_value="0.00"),
            DeclareLaunchArgument("ftg_gap_min_distance", default_value="0.00"),
            DeclareLaunchArgument("ftg_max_range", default_value="0.00"),
            DeclareLaunchArgument("ftg_turn_bias", default_value="0.00"),
            DeclareLaunchArgument("return_speed", default_value="0.06"),
            DeclareLaunchArgument("return_turn", default_value="0.80"),
            DeclareLaunchArgument("return_cross_track_kp", default_value="4.00"),
            DeclareLaunchArgument("return_yaw_kp", default_value="1.20"),
            DeclareLaunchArgument("scan_yaw_offset_deg", default_value="-90.0"),
            DeclareLaunchArgument("angular_sign", default_value="-1.0"),
            DeclareLaunchArgument("junction_stop_enabled", default_value="false"),
            DeclareLaunchArgument("junction_front_open_distance", default_value="0.80"),
            DeclareLaunchArgument("junction_side_open_distance", default_value="1.00"),
            DeclareLaunchArgument("junction_side_min_distance", default_value="0.90"),
            DeclareLaunchArgument("junction_open_ratio_threshold", default_value="0.72"),
            DeclareLaunchArgument("junction_required_open_paths", default_value="3"),
            DeclareLaunchArgument("junction_confirm_cycles", default_value="6"),
            DeclareLaunchArgument("junction_side_start_deg", default_value="55.0"),
            DeclareLaunchArgument("junction_side_end_deg", default_value="110.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sllidar_launch)),
                launch_arguments={'serial_port': '/dev/sllidar'}.items()
            ),
            TimerAction(
                period=4.0,
                actions=[
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ekf_launch))),
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '-1.5708', '0', '0', 'base_footprint', 'laser'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(avoid_script),
                    "--scan-topic",
                    scan_topic,
                    "--odom-topic",
                    odom_topic,
                    "--input-topic",
                    cmd_raw_topic,
                    "--output-topic",
                    cmd_safe_topic,
                    "--front-stop-distance",
                    front_stop_distance,
                    "--front-clear-distance",
                    front_clear_distance,
                    "--lateral-stop-distance",
                    lateral_stop_distance,
                    "--avoid-release-distance",
                    avoid_release_distance,
                    "--emergency-stop-distance",
                    emergency_stop_distance,
                    "--avoid-speed",
                    avoid_speed,
                    "--avoid-speed-min",
                    avoid_speed_min,
                    "--avoid-turn",
                    avoid_turn,
                    "--avoid-turn-kp",
                    avoid_turn_kp,
                    "--ftg-bubble-radius-m",
                    ftg_bubble_radius_m,
                    "--ftg-gap-min-distance",
                    ftg_gap_min_distance,
                    "--ftg-max-range",
                    ftg_max_range,
                    "--ftg-turn-bias",
                    ftg_turn_bias,
                    "--return-speed",
                    return_speed,
                    "--return-turn",
                    return_turn,
                    "--return-cross-track-kp",
                    return_cross_track_kp,
                    "--return-yaw-kp",
                    return_yaw_kp,
                    "--junction-stop-enabled",
                    junction_stop_enabled,
                    "--junction-front-open-distance",
                    junction_front_open_distance,
                    "--junction-side-open-distance",
                    junction_side_open_distance,
                    "--junction-side-min-distance",
                    junction_side_min_distance,
                    "--junction-open-ratio-threshold",
                    junction_open_ratio_threshold,
                    "--junction-required-open-paths",
                    junction_required_open_paths,
                    "--junction-confirm-cycles",
                    junction_confirm_cycles,
                    "--junction-side-start-deg",
                    junction_side_start_deg,
                    "--junction-side-end-deg",
                    junction_side_end_deg,
                    "--scan-yaw-offset-deg",
                    scan_yaw_offset_deg,
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(bridge_script),
                    "--port",
                    port,
                    "--baud",
                    baud,
                    "--wheel-base",
                    wheel_base,
                    "--cmd-topic",
                    cmd_safe_topic,
                    "--action-topic",
                    action_topic,
                    "--heading-offset-topic",
                    heading_offset_topic,
                    "--angular-sign",
                    angular_sign,
                    "--odom-linear-scale",
                    "0.04",
                    "--odom-velocity-scale",
                    "0.04",
                ],
                output="screen",
            ),
                ]
            ),
        ]
    )
