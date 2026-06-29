#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "用法: $0 <normal_S> [fine_S]"
  echo "示例: $0 2 1"
  echo "说明: 只给一个值时，仅设置正常模式 normal_S。"
  exit 2
fi

NORMAL_S="$1"
FINE_S="${2:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

TELEOP_SERVER="${TELEOP_SERVER_NODE:-/spacemouse_franka_impedance_teleop_server}"

if ! ros2 param list "${TELEOP_SERVER}" 2>/dev/null | grep -Fxq "  normal_spacemouse_target_scale"; then
  echo "当前 teleop server 未声明 normal_spacemouse_target_scale。"
  echo "通常原因是 Teleop 仍是旧进程。请在 UI 中 Clear 后重新启动 Teleop，再运行本命令。"
  exit 3
fi

ros2 param set "${TELEOP_SERVER}" normal_spacemouse_target_scale "${NORMAL_S}"
if [ -n "${FINE_S}" ]; then
  ros2 param set "${TELEOP_SERVER}" fine_spacemouse_target_scale "${FINE_S}"
fi

echo "当前 SpaceMouse scale:"
ros2 param get "${TELEOP_SERVER}" normal_spacemouse_target_scale
ros2 param get "${TELEOP_SERVER}" fine_spacemouse_target_scale
ros2 param get "${TELEOP_SERVER}" spacemouse_target_scale
ros2 param get "${TELEOP_SERVER}" speed_scale
