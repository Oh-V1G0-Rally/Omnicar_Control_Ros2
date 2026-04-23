import os
import re
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _next_bag_path(bag_directory, test_name):
    os.makedirs(bag_directory, exist_ok=True)
    pattern = re.compile(rf"^{re.escape(test_name)}_(\d+)$")
    indices = []
    for entry in os.listdir(bag_directory):
        match = pattern.match(entry)
        if match:
            indices.append(int(match.group(1)))
    next_index = max(indices, default=0) + 1
    return os.path.join(bag_directory, f"{test_name}_{next_index:02d}")


def _ns_topic(robot_id, topic):
    return f"/{robot_id}/{topic}" if robot_id else f"/{topic}"


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    robot_id = LaunchConfiguration("robot_id").perform(context)
    control_frame_id = LaunchConfiguration("control_frame_id")
    initial_goal_frame_id = LaunchConfiguration("initial_goal_frame_id")
    record_bag = _as_bool(LaunchConfiguration("record_bag").perform(context))
    bag_directory = LaunchConfiguration("bag_directory").perform(context)
    test_name = LaunchConfiguration("test_name").perform(context)
    open_rqt_plot = _as_bool(LaunchConfiguration("open_rqt_plot_on_shutdown").perform(context))

    topics_to_record = [
        _ns_topic(robot_id, "go_to_point_debug"),
        _ns_topic(robot_id, "control_error"),
        _ns_topic(robot_id, "cmd_vel"),
        _ns_topic(robot_id, "cmd_vel_ref"),
        _ns_topic(robot_id, "goal_pose"),
        _ns_topic(robot_id, "goal_xyyaw"),
        _ns_topic(robot_id, "goal_reached"),
        _ns_topic(robot_id, "pose"),
        _ns_topic(robot_id, "odom"),
        "/tf",
        "/tf_static",
    ]

    plot_topics = [
        _ns_topic(robot_id, "go_to_point_debug/error_x_body"),
        _ns_topic(robot_id, "go_to_point_debug/error_y_body"),
        _ns_topic(robot_id, "go_to_point_debug/error_yaw"),
        _ns_topic(robot_id, "go_to_point_debug/cmd_vx"),
        _ns_topic(robot_id, "go_to_point_debug/cmd_vy"),
        _ns_topic(robot_id, "go_to_point_debug/cmd_w"),
        _ns_topic(robot_id, "go_to_point_debug/distance_error"),
    ]

    actions = [
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
                    "pose_source": "pose",
                    "control_frame_id": control_frame_id,
                    "initial_goal_frame_id": initial_goal_frame_id,
                },
            ],
        ),
    ]

    bag_path = None
    if record_bag:
        bag_path = _next_bag_path(bag_directory, test_name)
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", bag_path] + topics_to_record,
                output="screen",
            )
        )

    if open_rqt_plot:
        if bag_path:
            rqt_plot_cmd = (
                "setsid bash -lc "
                + shlex.quote(
                    "nohup rqt_plot "
                    + " ".join(plot_topics)
                    + " >/tmp/go_to_point_rqt_plot.log 2>&1 & "
                    + "sleep 3; "
                    + "ros2 bag play "
                    + shlex.quote(bag_path)
                    + " >/tmp/go_to_point_bag_play.log 2>&1"
                )
                + " >/tmp/go_to_point_post_test.log 2>&1 &"
            )
        else:
            rqt_plot_cmd = (
                "setsid rqt_plot "
                + " ".join(plot_topics)
                + " >/tmp/go_to_point_rqt_plot.log 2>&1 &"
            )
        actions.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        ExecuteProcess(
                            cmd=["bash", "-lc", rqt_plot_cmd],
                            output="screen",
                        )
                    ]
                )
            )
        )

    return actions


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    control_frame_id = LaunchConfiguration("control_frame_id")
    initial_goal_frame_id = LaunchConfiguration("initial_goal_frame_id")

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
            "initial_goal_frame_id",
            default_value=[robot_id, "/map"],
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
            default_value="go_to_point_map",
        ),
        DeclareLaunchArgument(
            "open_rqt_plot_on_shutdown",
            default_value="true",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
