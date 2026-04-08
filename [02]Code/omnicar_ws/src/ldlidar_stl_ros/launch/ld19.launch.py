from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    port_name = LaunchConfiguration("port_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/ttyUSB1",
        ),
        Node(
            package="ldlidar_stl_ros",
            executable="ldlidar_stl_ros_node",
            name="LD19",
            namespace=robot_id,
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("ldlidar_stl_ros"),
                    "config",
                    "sdpo_driver_laser_2d_LD19.yaml",
                ]),
                {
                    "port_name": port_name,
                    "base_frame_id": [robot_id, "/base_footprint"],
                    "laser_frame_id": [robot_id, "/laser"],
                },
            ],
        ),
    ])
