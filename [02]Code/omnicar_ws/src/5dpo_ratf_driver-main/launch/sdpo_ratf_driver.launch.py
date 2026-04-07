from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        Node(
            package="sdpo_ratf_driver",
            executable="sdpo_ratf_driver_node",
            namespace=robot_id,
            name="sdpo_ratf_driver",
            parameters=[
                {"encoder_res": 64.0},
                {"gear_reduction": 18.75},
                {"serial_port_name": "/dev/omnicar_esp32"},
                {"mot_ctrl_freq": 50},
                {"max_mot_pwm": 1023},
            ],
        ),
    ])
