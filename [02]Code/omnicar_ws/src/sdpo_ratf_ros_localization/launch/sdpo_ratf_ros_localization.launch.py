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
            package="sdpo_ratf_ros_localization",
            executable="sdpo_ratf_ros_localization_node",
            name="sdpo_ratf_ros_localization",
            namespace=robot_id,
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("sdpo_ratf_ros_localization"),
                    "config",
                    "sdpo_ratf_ros_localization.yaml",
                ]),
                {
                    "map_frame_id": [robot_id, "/map"],
                    "odom_frame_id": [robot_id, "/odom"],
                    "base_frame_id": [robot_id, "/base_footprint"],
                    "laser_frame_id": [robot_id, "/laser"],
                },
            ],
        ),
    ])
