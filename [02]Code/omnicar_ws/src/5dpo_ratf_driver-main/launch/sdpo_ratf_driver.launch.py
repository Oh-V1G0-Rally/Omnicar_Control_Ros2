from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    serial_port_name = LaunchConfiguration("serial_port_name")
    encoder_res = LaunchConfiguration("encoder_res")
    gear_reduction = LaunchConfiguration("gear_reduction")
    mot_ctrl_freq = LaunchConfiguration("mot_ctrl_freq")
    max_mot_pwm = LaunchConfiguration("max_mot_pwm")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value=EnvironmentVariable("ROBOT_ID", default_value="unnamed_robot"),
        ),
        DeclareLaunchArgument(
            "serial_port_name",
            default_value="/dev/omnicar_esp32",
        ),
        DeclareLaunchArgument(
            "encoder_res",
            default_value="64.0",
        ),
        DeclareLaunchArgument(
            "gear_reduction",
            default_value="18.75",
        ),
        DeclareLaunchArgument(
            "mot_ctrl_freq",
            default_value="50",
        ),
        DeclareLaunchArgument(
            "max_mot_pwm",
            default_value="1023",
        ),
        Node(
            package="sdpo_ratf_driver",
            executable="sdpo_ratf_driver_node",
            namespace=robot_id,
            name="sdpo_ratf_driver",
            parameters=[
                {"encoder_res": encoder_res},
                {"gear_reduction": gear_reduction},
                {"serial_port_name": serial_port_name},
                {"mot_ctrl_freq": mot_ctrl_freq},
                {"max_mot_pwm": max_mot_pwm},
            ],
        ),
    ])
