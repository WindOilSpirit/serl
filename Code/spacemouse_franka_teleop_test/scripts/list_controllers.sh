#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

echo "Available controller types:"
if ! timeout 8 ros2 control list_controller_types \
  | grep -E "serl|franka_example|franka_robot_state|joint_state_broadcaster"; then
  echo "controller_manager 不可用，无法查询 controller types。"
fi
echo
echo "Loaded controllers:"
if ! timeout 8 ros2 control list_controllers; then
  echo "controller_manager 不可用，无法查询 loaded controllers。" >&2
  exit 6
fi
