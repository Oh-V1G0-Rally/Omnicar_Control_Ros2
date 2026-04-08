from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    use_driver = LaunchConfiguration("use_driver")
    use_joy = LaunchConfiguration("use_joy")
    use_odom = LaunchConfiguration("use_odom")
    use_cmd_vel_bridge = LaunchConfiguration("use_cmd_vel_bridge")
    use_lidar = LaunchConfiguration("use_lidar")
    use_localization = LaunchConfiguration("use_localization")
    driver_port = LaunchConfiguration("driver_port")
    lidar_port = LaunchConfiguration("lidar_port")
    joy_dev = LaunchConfiguration("joy_dev")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        DeclareLaunchArgument("use_driver", default_value="true"),
        DeclareLaunchArgument("use_joy", default_value="true"),
        DeclareLaunchArgument("use_odom", default_value="true"),
        DeclareLaunchArgument("use_cmd_vel_bridge", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_localization", default_value="true"),
        DeclareLaunchArgument("driver_port", default_value="/dev/omnicar_esp32"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ratf_driver"),
                    "launch",
                    "sdpo_ratf_driver.launch.py",
                ])
            ),
            launch_arguments={
                "robot_id": robot_id,
                "serial_port_name": driver_port,
            }.items(),
            condition=IfCondition(use_driver),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_driver_omnijoy"),
                    "launch",
                    "sdpo_driver_omnijoy_logif710.launch.py",
                ])
            ),
            launch_arguments={
                "robot_id": robot_id,
                "joy_dev": joy_dev,
            }.items(),
            condition=IfCondition(use_joy),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ros_odom"),
                    "launch",
                    "sdpo_ros_odom_cmd_vel_bridge.launch.py",
                ])
            ),
            launch_arguments={"robot_id": robot_id}.items(),
            condition=IfCondition(use_cmd_vel_bridge),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ros_odom"),
                    "launch",
                    "sdpo_ros_odom_wh.launch.py",
                ])
            ),
            launch_arguments={"robot_id": robot_id}.items(),
            condition=IfCondition(use_odom),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ldlidar_stl_ros"),
                    "launch",
                    "ld19.launch.py",
                ])
            ),
            launch_arguments={
                "robot_id": robot_id,
                "port_name": lidar_port,
            }.items(),
            condition=IfCondition(use_lidar),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ratf_ros_localization"),
                    "launch",
                    "sdpo_ratf_ros_localization.launch.py",
                ])
            ),
            launch_arguments={"robot_id": robot_id}.items(),
            condition=IfCondition(use_localization),
        ),
    ])
