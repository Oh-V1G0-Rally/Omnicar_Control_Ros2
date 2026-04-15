from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    pose_source = LaunchConfiguration("pose_source")
    control_frame_id = LaunchConfiguration("control_frame_id")
    initial_goal_frame_id = LaunchConfiguration("initial_goal_frame_id")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        DeclareLaunchArgument(
            "pose_source",
            default_value="pose",
        ),
        DeclareLaunchArgument(
            "control_frame_id",
            default_value=[robot_id, "/map"],
        ),
        DeclareLaunchArgument(
            "initial_goal_frame_id",
            default_value=[robot_id, "/map"],
        ),
        Node(
            package="sdpo_motion_control",
            executable="go_to_point_controller_node",
            name="go_to_point_controller",
            namespace=robot_id,
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("sdpo_motion_control"),
                    "config",
                    "go_to_point_controller.yaml",
                ]),
                {
                    "pose_source": pose_source,
                    "control_frame_id": control_frame_id,
                    "initial_goal_frame_id": initial_goal_frame_id,
                },
            ],
        ),
    ])
