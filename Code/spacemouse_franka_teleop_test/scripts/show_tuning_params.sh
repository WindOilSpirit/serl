#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

TELEOP_SERVER="${TELEOP_SERVER_NODE:-/spacemouse_franka_impedance_teleop_server}"
CONTROLLER="${SERL_CONTROLLER_NODE:-/serl_cartesian_impedance_controller}"

echo "SpaceMouse target scale:"
if ros2 param list "${TELEOP_SERVER}" 2>/dev/null | grep -Fxq "  normal_spacemouse_target_scale"; then
  echo "normal:"
  ros2 param get "${TELEOP_SERVER}" normal_spacemouse_target_scale
  echo "fine:"
  ros2 param get "${TELEOP_SERVER}" fine_spacemouse_target_scale
  echo "global multiplier:"
  ros2 param get "${TELEOP_SERVER}" spacemouse_target_scale
  echo "base speed_scale:"
  ros2 param get "${TELEOP_SERVER}" speed_scale
else
  echo "teleop server 未声明新 scale 参数；请重启 Teleop。"
  echo "旧参数 speed_scale:"
  ros2 param get "${TELEOP_SERVER}" speed_scale || true
fi
echo
echo "Controller translational stiffness K:"
ros2 param get "${CONTROLLER}" translational_stiffness
