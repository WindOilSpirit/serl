#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <controller_name>" >&2
  exit 2
fi

source /home/admin123/WenshuoZhou/SERL/serl-main/scripts/source_ros2_franka_env.sh

ros2 run controller_manager unspawner "$1" \
  -c /controller_manager

