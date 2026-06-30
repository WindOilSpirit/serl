#!/usr/bin/env bash
set -eo pipefail

source /home/admin123/WenshuoZhou/SERL/hil-serl-main/scripts/source_ros2_franka_env.sh

ros2 launch franka_bringup franka.launch.py \
  arm_id:=fr3 \
  robot_ip:=172.16.0.2

