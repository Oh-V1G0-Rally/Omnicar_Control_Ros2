from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    state_input_type = LaunchConfiguration("state_input_type")
    control_frame_id = LaunchConfiguration("control_frame_id")
    initial_goal_frame_id = LaunchConfiguration("initial_goal_frame_id")
    state_topic = LaunchConfiguration("state_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        DeclareLaunchArgument(
            "state_input_type",
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
        DeclareLaunchArgument(
            "state_topic",
            default_value="motion_state",
        ),
        Node(
            package="sdpo_motion_control",
            executable="motion_state_adapter_node",
            name="motion_state_adapter",
            namespace=robot_id,
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("sdpo_motion_control"),
                    "config",
                    "motion_state_adapter.yaml",
                ]),
                {
                    "input_type": state_input_type,
                    "state_topic": state_topic,
                    "default_state_frame_id": control_frame_id,
                },
            ],
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
                    "state_topic": state_topic,
                    "control_frame_id": control_frame_id,
                    "initial_goal_frame_id": initial_goal_frame_id,
                },
            ],
        ),
    ])
