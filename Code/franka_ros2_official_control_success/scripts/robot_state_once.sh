#!/usr/bin/env bash
set -eo pipefail

source /home/admin123/WenshuoZhou/SERL/serl-main/scripts/source_ros2_franka_env.sh

ros2 topic echo /franka_robot_state_broadcaster/robot_state --once

