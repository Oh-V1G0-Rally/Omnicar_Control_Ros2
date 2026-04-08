from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    joy_dev = LaunchConfiguration("joy_dev")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="driver_joy",
            namespace=robot_id,
            output="screen",
            parameters=[
                {"dev": joy_dev},
                {"autorepeat_rate": 10.0},
                {"coalesce_interval": 0.05},
            ],
        ),
        Node(
            package="sdpo_driver_omnijoy",
            executable="sdpo_driver_omnijoy_node",
            name="sdpo_driver_omnijoy",
            namespace=robot_id,
            output="screen",
            parameters=[
                {"axis_linear_x": 1},
                {"axis_linear_y": 0},
                {"axis_angular": 2},
                {"axis_deadman": 4},
                {"axis_turbo": 5},
                {"axis_turbo_up": 7},
                {"axis_turbo_down": 6},
                {"scale_angular": 1.0},
                {"scale_linear": 0.4},
                {"turbo_scale_linear": 0.25},
                {"turbo_max_scale_linear": 0.4},
                {"turbo_scale_angular": 0.4},
            ],
        ),
    ])
