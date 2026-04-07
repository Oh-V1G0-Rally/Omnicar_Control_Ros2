from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ratf_driver"),
                    "launch",
                    "sdpo_ratf_driver.launch.py",
                ])
            ),
            launch_arguments={"robot_id": robot_id}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_driver_omnijoy"),
                    "launch",
                    "sdpo_driver_omnijoy_logif710.launch.py",
                ])
            ),
            launch_arguments={"robot_id": robot_id}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ros_odom"),
                    "launch",
                    "sdpo_ros_odom_cmd_vel.launch.py",
                ])
            ),
            launch_arguments={"robot_id": robot_id}.items(),
        ),
    ])
