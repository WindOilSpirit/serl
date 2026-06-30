#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/source_ros2_franka_env.sh"

LOG_DIR="${SPACEMOUSE_FRANKA_LOG_DIR:-/tmp/spacemouse_franka_teleop_logs}"
mkdir -p "${LOG_DIR}"

if [ -n "${SERL_CONTROLLER_START_CMD:-}" ]; then
  echo "执行 SERL_CONTROLLER_START_CMD:"
  echo "  ${SERL_CONTROLLER_START_CMD}"
  exec bash -lc "${SERL_CONTROLLER_START_CMD}"
fi

ARM_ID="${FRANKA_ARM_ID:-fr3}"
ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
CONTROLLER_MANAGER="${SERL_CONTROLLER_MANAGER:-/controller_manager}"
CONTROLLER_NAME="${SERL_CONTROLLER_NAME:-serl_cartesian_impedance_controller}"
CONTROLLER_TYPE="${SERL_CONTROLLER_TYPE:-serl_franka_ros2_control/SerlCartesianImpedanceController}"
CONTROLLER_PARAM_FILE="${SERL_CONTROLLER_PARAM_FILE:-}"
BRINGUP_LOG="${LOG_DIR}/franka_bringup_from_controller_button.log"

bringup_pid=""
started_bringup=0
controller_manager_ready() {
  timeout 2 ros2 control list_controllers -c "${CONTROLLER_MANAGER}" >/dev/null 2>&1
}

stop_stale_bringup() {
  echo "清理不可用的 Franka bringup/controller_manager 残留进程。"
  pkill -INT -f "[r]os2 launch franka_bringup" || true
  pkill -INT -f "[f]ranka.launch.py" || true
  pkill -INT -f "[r]os2_control_node" || true
  sleep 2
  pkill -TERM -f "[r]os2 launch franka_bringup" || true
  pkill -TERM -f "[f]ranka.launch.py" || true
  pkill -TERM -f "[r]os2_control_node" || true
  sleep 1
}

start_bringup() {
  echo "启动 Franka bringup，日志: ${BRINGUP_LOG}"
  ros2 launch franka_bringup franka.launch.py \
    arm_id:="${ARM_ID}" \
    robot_ip:="${ROBOT_IP}" \
    >"${BRINGUP_LOG}" 2>&1 &
  bringup_pid="$!"
  started_bringup=1
}

cleanup() {
  if [ -n "${bringup_pid}" ] && kill -0 "${bringup_pid}" 2>/dev/null; then
    kill -INT "${bringup_pid}" 2>/dev/null || true
    wait "${bringup_pid}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

if pgrep -af "[r]os2_control_node|[f]ranka.launch.py" >/dev/null; then
  echo "检测到已有 Franka bringup/controller_manager 相关进程，检查 controller_manager 可用性。"
  if controller_manager_ready; then
    echo "已有 controller_manager 可用，跳过重复启动。"
  else
    echo "已有进程但 controller_manager 不可用，将重启 Franka bringup。"
    stop_stale_bringup
    start_bringup
  fi
else
  start_bringup
fi

echo "等待 controller_manager 可用: ${CONTROLLER_MANAGER}"
manager_ready=0
for _ in $(seq 1 30); do
  if controller_manager_ready; then
    manager_ready=1
    break
  fi
  sleep 1
done

if [ "${manager_ready}" != "1" ]; then
  echo "controller_manager 不可用: ${CONTROLLER_MANAGER}" >&2
  echo "请查看 ${BRINGUP_LOG}，并确认 franka_bringup/ros2_control_node 已成功启动。" >&2
  exit 6
fi

echo "启动 controller spawner: ${CONTROLLER_NAME}"
if [ -z "${CONTROLLER_PARAM_FILE}" ]; then
  if controller_prefix="$(ros2 pkg prefix serl_franka_ros2_control 2>/dev/null)"; then
    candidate_param_file="${controller_prefix}/share/serl_franka_ros2_control/config/serl_cartesian_impedance_controller.yaml"
    if [ -f "${candidate_param_file}" ]; then
      CONTROLLER_PARAM_FILE="${candidate_param_file}"
    fi
  fi
fi

spawner_args=(
  "${CONTROLLER_NAME}"
  -c "${CONTROLLER_MANAGER}"
  --controller-manager-timeout 15
  --service-call-timeout 15
  --switch-timeout 15
)
if [ -n "${CONTROLLER_TYPE}" ]; then
  spawner_args+=(-t "${CONTROLLER_TYPE}")
fi
if [ -n "${CONTROLLER_PARAM_FILE}" ]; then
  echo "使用 controller 参数文件: ${CONTROLLER_PARAM_FILE}"
  spawner_args+=(--param-file "${CONTROLLER_PARAM_FILE}")
else
  echo "未找到 SERL controller 参数文件；将只按 controller type 加载。"
fi

if [ -n "${CONTROLLER_TYPE}" ]; then
  controller_types_output="$(mktemp /tmp/serl_controller_types.XXXXXX)"
  if timeout 8 ros2 control list_controller_types -c "${CONTROLLER_MANAGER}" >"${controller_types_output}" 2>&1; then
    if ! grep -Fq "${CONTROLLER_TYPE}" "${controller_types_output}"; then
      echo "警告：当前 controller_manager 的 list_controller_types 未列出: ${CONTROLLER_TYPE}" >&2
      echo "仍会继续调用 spawner -t 尝试加载；若失败，请检查 overlay 是否已 source。" >&2
    fi
  else
    echo "警告：无法读取 controller types，仍会继续调用 spawner -t 尝试加载。" >&2
    sed -n '1,80p' "${controller_types_output}" >&2 || true
  fi
  rm -f "${controller_types_output}"
  if [ "${started_bringup}" != "1" ]; then
    if ! timeout 8 ros2 control list_controller_types -c "${CONTROLLER_MANAGER}" 2>/dev/null | grep -Fq "${CONTROLLER_TYPE}"; then
      echo "提示：检测到复用了已有 controller_manager；如果 spawner 失败，请先按 Clear 停止 ROS/teleop 进程，再按 Controller 让 franka_bringup 在新环境中重启。" >&2
    fi
  fi
fi

if ! ros2 run controller_manager spawner "${spawner_args[@]}"; then
  echo "controller spawner 失败: ${CONTROLLER_NAME}" >&2
  echo "若 list_controller_types 中没有新 controller plugin，请先构建/安装 ROS2 版 serl_cartesian_impedance_controller。" >&2
  if [ "${started_bringup}" != "1" ]; then
    echo "检测到复用了已有 controller_manager；它可能是在构建/source 新 overlay 之前启动的。" >&2
    echo "请先按 Clear 停止 ROS/teleop 进程，再按 Controller 让 franka_bringup 在新环境中重启。" >&2
  fi
  exit 7
fi

echo "controller 已完成 load/configure/activate 请求。"
echo "保持 Franka bringup 运行；按 Ctrl-C 或使用 Clear 停止。"
if [ -n "${bringup_pid}" ]; then
  wait "${bringup_pid}"
else
  while true; do
    sleep 1
  done
fi
