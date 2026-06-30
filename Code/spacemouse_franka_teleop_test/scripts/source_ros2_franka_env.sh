#!/usr/bin/env bash
# 准备当前 teleop_test 使用的 ROS2 + franka_ros2 环境。

set -eo pipefail

SERL_WORKSPACE="${SERL_WORKSPACE:-/home/admin123/WenshuoZhou/SERL}"
ROS2_WS="${ROS2_WS:-/home/admin123/ros2_ws}"

strip_path_entries() {
  local value="${1:-}"
  local filtered=""
  local entry
  IFS=':' read -ra entries <<< "${value}"
  for entry in "${entries[@]}"; do
    if [[ -n "${entry}" && "${entry}" != *"/miniforge3"* && "${entry}" != *"/anaconda"* && "${entry}" != *"/conda"* ]]; then
      if [[ -z "${filtered}" ]]; then
        filtered="${entry}"
      else
        filtered="${filtered}:${entry}"
      fi
    fi
  done
  printf '%s' "${filtered}"
}

strip_missing_path_entries() {
  local value="${1:-}"
  local filtered=""
  local entry
  IFS=':' read -ra entries <<< "${value}"
  for entry in "${entries[@]}"; do
    if [[ -n "${entry}" && -e "${entry}" ]]; then
      if [[ -z "${filtered}" ]]; then
        filtered="${entry}"
      else
        filtered="${filtered}:${entry}"
      fi
    fi
  done
  printf '%s' "${filtered}"
}

export PATH="$(strip_path_entries "${PATH:-}")"
export LD_LIBRARY_PATH="$(strip_path_entries "${LD_LIBRARY_PATH:-}")"
export CMAKE_PREFIX_PATH="$(strip_path_entries "${CMAKE_PREFIX_PATH:-}")"
export PYTHONNOUSERSITE=1
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset CONDA_TOOLCHAIN_BUILD CONDA_TOOLCHAIN_HOST CONDA_BUILD_SYSROOT
unset _CONDA_PYTHON_SYSCONFIGDATA_NAME PYTHONHOME
if [[ "${CC:-}" == *"/miniforge3/"* || "${CC:-}" == *"/anaconda"* || "${CC:-}" == *"/conda"* ]]; then
  unset CC
fi
if [[ "${CXX:-}" == *"/miniforge3/"* || "${CXX:-}" == *"/anaconda"* || "${CXX:-}" == *"/conda"* ]]; then
  unset CXX
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/spacemouse_franka_teleop_ros_logs}"
export ROS_DISABLE_DAEMON="${ROS_DISABLE_DAEMON:-1}"
mkdir -p "${ROS_LOG_DIR}"

source /opt/ros/humble/setup.bash

if [ -f "${ROS2_WS}/install/local_setup.bash" ]; then
  source "${ROS2_WS}/install/local_setup.bash"
else
  echo "未找到 Franka ROS2 workspace: ${ROS2_WS}/install/local_setup.bash" >&2
  return 2 2>/dev/null || exit 2
fi

if [ -f "${SERL_WORKSPACE}/install/local_setup.bash" ]; then
  source "${SERL_WORKSPACE}/install/local_setup.bash"
fi

export AMENT_PREFIX_PATH="$(strip_missing_path_entries "${AMENT_PREFIX_PATH:-}")"
export COLCON_PREFIX_PATH="$(strip_missing_path_entries "${COLCON_PREFIX_PATH:-}")"
export LD_LIBRARY_PATH="$(strip_missing_path_entries "${LD_LIBRARY_PATH:-}")"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/usr/local/lib:${LD_LIBRARY_PATH:-}"

# The current /home/admin123/ros2_ws franka_hardware build references fmt::v12
# but does not declare libfmt.so.12 as a direct NEEDED dependency. Preload only
# that library instead of reintroducing the full conda runtime into LD_LIBRARY_PATH.
FMT12_LIBRARY="${FMT12_LIBRARY:-/home/admin123/miniforge3/lib/libfmt.so.12}"
if [ -f "${FMT12_LIBRARY}" ]; then
  case ":${LD_PRELOAD:-}:" in
    *":${FMT12_LIBRARY}:"*) ;;
    *) export LD_PRELOAD="${FMT12_LIBRARY}${LD_PRELOAD:+:${LD_PRELOAD}}" ;;
  esac
fi

if [ -f "${SERL_WORKSPACE}/.venv/bin/activate" ]; then
  source "${SERL_WORKSPACE}/.venv/bin/activate"
fi

export SERL_WORKSPACE
export ROS2_WS
