from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        DeclareLaunchArgument(
            "use_driver",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_joy",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_odom",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_cmd_vel_bridge",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_localization",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/omnicar_lidar",
        ),
        DeclareLaunchArgument(
            "driver_port",
            default_value="/dev/omnicar_esp32",
        ),
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ratf_driver"),
                    "launch",
                    "omnicar_bringup.launch.py",
                ])
            ),
            launch_arguments={
                "robot_id": robot_id,
                "use_driver": use_driver,
                "use_joy": use_joy,
                "use_odom": use_odom,
                "use_cmd_vel_bridge": use_cmd_vel_bridge,
                "use_lidar": use_lidar,
                "use_localization": use_localization,
                "driver_port": driver_port,
                "lidar_port": lidar_port,
                "joy_dev": joy_dev,
            }.items(),
        ),
    ])
