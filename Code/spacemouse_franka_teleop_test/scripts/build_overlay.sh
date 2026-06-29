#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"
CONTROLLER_DIR="${WS_DIR}/serl_franka_ros2_control"

source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

cd "${WS_DIR}"
colcon build \
  --base-paths "${PKG_DIR}" "${CONTROLLER_DIR}" \
  --packages-select spacemouse_franka_teleop_test serl_franka_ros2_control \
  --symlink-install \
  --cmake-clean-cache

echo
echo "构建完成。请运行："
echo "  source ${PKG_DIR}/scripts/source_ros2_franka_env.sh"
