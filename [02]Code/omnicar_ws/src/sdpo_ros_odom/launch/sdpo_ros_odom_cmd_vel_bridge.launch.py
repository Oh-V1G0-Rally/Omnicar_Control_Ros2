from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        Node(
            package="sdpo_ros_odom",
            executable="sdpo_ros_odom_cmd_vel_node",
            name="sdpo_ros_odom_cmd_vel",
            namespace=robot_id,
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ros_odom"),
                    "config",
                    "sdpo_ros_odom_cmd_vel.yaml",
                ])
            ],
        ),
    ])
