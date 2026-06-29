#!/usr/bin/env bash
# Source this before running ROS2/Franka/SERL integration commands.
#
# This repository's original Franka server is ROS1-based, but the current
# machine has ROS2 Humble + franka_ros2 installed. This script prepares the
# ROS2 Franka environment and keeps ROS logs inside the writable workspace.

set -e

SERL_WORKSPACE="/home/admin123/WenshuoZhou/SERL"
ROS2_WS="/home/admin123/ros2_ws"

# ROS2 control plugins are loaded into controller_manager, so they must use the
# system C++ runtime. Strip conda/miniforge paths that can shadow libstdc++.
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

export PATH="$(strip_path_entries "${PATH:-}")"
export LD_LIBRARY_PATH="$(strip_path_entries "${LD_LIBRARY_PATH:-}")"
export CMAKE_PREFIX_PATH="$(strip_path_entries "${CMAKE_PREFIX_PATH:-}")"
export PYTHONNOUSERSITE=1
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset CONDA_TOOLCHAIN_BUILD CONDA_TOOLCHAIN_HOST CONDA_BUILD_SYSROOT
unset _CONDA_PYTHON_SYSCONFIGDATA_NAME PYTHONHOME

export ROS_LOG_DIR="${SERL_WORKSPACE}/serl-main/ros_logs"
mkdir -p "${ROS_LOG_DIR}"

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
if [ -f "${SERL_WORKSPACE}/serl-main/ros2_control_ws/install/setup.bash" ]; then
  source "${SERL_WORKSPACE}/serl-main/ros2_control_ws/install/setup.bash"
fi
if [ -f "/tmp/franka_circle_ws/install/setup.bash" ]; then
  source "/tmp/franka_circle_ws/install/setup.bash"
fi
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
source "${SERL_WORKSPACE}/.venv/bin/activate"

export SERL_WORKSPACE
export ROS2_WS

echo "ROS_DISTRO=${ROS_DISTRO}"
echo "ROS_VERSION=${ROS_VERSION}"
echo "ROS_LOG_DIR=${ROS_LOG_DIR}"
echo "VIRTUAL_ENV=${VIRTUAL_ENV}"
