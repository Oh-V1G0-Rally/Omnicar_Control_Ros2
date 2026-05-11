from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    control_frame_id = LaunchConfiguration("control_frame_id")
    heading_mode = LaunchConfiguration("heading_mode")
    path_file = LaunchConfiguration("path_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        DeclareLaunchArgument(
            "control_frame_id",
            default_value=[robot_id, "/map"],
        ),
        DeclareLaunchArgument(
            "heading_mode",
            default_value="fixed",
        ),
        DeclareLaunchArgument(
            "path_file",
            default_value="/home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/src/splines/FEUP/FEUP_rounded_rect_eight.csv",
        ),
        Node(
            package="sdpo_motion_control",
            executable="nonlinear_mpcc_controller_node",
            name="nonlinear_mpcc_controller",
            namespace=robot_id,
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("sdpo_motion_control"),
                    "config",
                    "nonlinear_mpcc_controller.yaml",
                ]),
                {
                    "control_frame_id": control_frame_id,
                    "heading_mode": heading_mode,
                    "path_file": path_file,
                },
            ],
        ),
    ])
