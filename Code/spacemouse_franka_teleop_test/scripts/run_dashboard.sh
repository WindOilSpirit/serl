#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

export SPACEMOUSE_FRANKA_TELEOP_DIR="${PKG_DIR}"

exec ros2 run spacemouse_franka_teleop_test teleop_dashboard --ros-args \
  --params-file "${PKG_DIR}/config/teleop_params.yaml"
