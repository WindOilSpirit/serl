#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

echo "启动 SpaceMouse -> SERL Cartesian Impedance teleop。"
echo "底层 controller 应已 active: serl_cartesian_impedance_controller"
exec ros2 launch spacemouse_franka_teleop_test spacemouse_franka_teleop.launch.py
