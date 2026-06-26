#!/usr/bin/env bash
set -eo pipefail

# Minimal boot bringup for manual demos:
# ROS2 driver + cmd_vel-to-wheel bridge + wheel odometry + joystick teleop.

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
OMNICAR_WS="${OMNICAR_WS:-/home/c2sr/Omnicar_Control_Ros2/[02]Code/omnicar_ws}"

ROBOT_ID="${ROBOT_ID:-omnicar}"
DRIVER_PORT="${DRIVER_PORT:-/dev/omnicar_esp32}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
LIDAR_PORT="${LIDAR_PORT:-/dev/omnicar_lidar}"

USE_DRIVER="${USE_DRIVER:-true}"
USE_JOY="${USE_JOY:-true}"
USE_ODOM="${USE_ODOM:-true}"
USE_CMD_VEL_BRIDGE="${USE_CMD_VEL_BRIDGE:-true}"
USE_LIDAR="${USE_LIDAR:-false}"
USE_LOCALIZATION="${USE_LOCALIZATION:-false}"
USE_WAYPOINT_CONTROLLER="${USE_WAYPOINT_CONTROLLER:-false}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${OMNICAR_WS}/install/setup.bash"

exec ros2 launch sdpo_ratf_driver omnicar_joy_control.launch.py \
  robot_id:="${ROBOT_ID}" \
  driver_port:="${DRIVER_PORT}" \
  joy_dev:="${JOY_DEV}" \
  lidar_port:="${LIDAR_PORT}" \
  use_driver:="${USE_DRIVER}" \
  use_joy:="${USE_JOY}" \
  use_odom:="${USE_ODOM}" \
  use_cmd_vel_bridge:="${USE_CMD_VEL_BRIDGE}" \
  use_lidar:="${USE_LIDAR}" \
  use_localization:="${USE_LOCALIZATION}" \
  use_waypoint_controller:="${USE_WAYPOINT_CONTROLLER}"
