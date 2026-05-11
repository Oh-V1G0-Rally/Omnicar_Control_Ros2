import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _next_bag_path(bag_directory, test_name):
    os.makedirs(bag_directory, exist_ok=True)
    pattern = re.compile(rf"^{re.escape(test_name)}(?:_(\d+))?$")
    indices = []
    exact_match_found = False
    for entry in os.listdir(bag_directory):
        match = pattern.match(entry)
        if not match:
            continue
        if match.group(1) is None:
            exact_match_found = True
        else:
            indices.append(int(match.group(1)))

    if not exact_match_found and not indices:
        return os.path.join(bag_directory, test_name)

    next_index = max(indices, default=0) + 1
    return os.path.join(bag_directory, f"{test_name}_{next_index:02d}")


def _ns_topic(robot_id, topic):
    return f"/{robot_id}/{topic}" if robot_id else f"/{topic}"


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    robot_id = LaunchConfiguration("robot_id").perform(context)
    control_frame_id = LaunchConfiguration("control_frame_id")
    heading_mode = LaunchConfiguration("heading_mode")
    path_file = LaunchConfiguration("path_file")
    record_bag = _as_bool(LaunchConfiguration("record_bag").perform(context))
    bag_directory = LaunchConfiguration("bag_directory").perform(context)
    test_name = LaunchConfiguration("test_name").perform(context)

    topics_to_record = [
        _ns_topic(robot_id, "path_follower_debug"),
        _ns_topic(robot_id, "path_follower_error"),
        _ns_topic(robot_id, "cmd_vel"),
        _ns_topic(robot_id, "pose"),
        _ns_topic(robot_id, "odom"),
        "/tf",
        "/tf_static",
    ]

    actions = [
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
    ]

    if record_bag:
        bag_path = _next_bag_path(bag_directory, test_name)
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", bag_path] + topics_to_record,
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")

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
        DeclareLaunchArgument(
            "record_bag",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "bag_directory",
            default_value=EnvironmentVariable(
                "OMNICAR_BAG_DIR",
                default_value="/home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws/bags",
            ),
        ),
        DeclareLaunchArgument(
            "test_name",
            default_value="PF_YYYY-MM-DD_INFIN_MAP_MPCC_V01_R01",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
