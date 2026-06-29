#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

ARM_ID="${FRANKA_ARM_ID:-fr3}"
ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"

echo "启动 Franka bringup / controller_manager"
echo "  arm_id=${ARM_ID}"
echo "  robot_ip=${ROBOT_IP}"
exec ros2 launch franka_bringup franka.launch.py \
  arm_id:="${ARM_ID}" \
  robot_ip:="${ROBOT_IP}"
