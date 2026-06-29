#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -ne 1 ]; then
  echo "用法: $0 <K_N_per_m>"
  echo "示例: $0 1200"
  exit 2
fi

K="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

CONTROLLER="${SERL_CONTROLLER_NODE:-/serl_cartesian_impedance_controller}"

ros2 param set "${CONTROLLER}" translational_stiffness "${K}"
ros2 param get "${CONTROLLER}" translational_stiffness
