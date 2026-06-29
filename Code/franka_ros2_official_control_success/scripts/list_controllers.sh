#!/usr/bin/env bash
set -eo pipefail

source /home/admin123/WenshuoZhou/SERL/serl-main/scripts/source_ros2_franka_env.sh

echo "Available Franka example controller types:"
ros2 control list_controller_types | grep franka_example_controllers || true

echo
echo "Loaded controllers:"
ros2 control list_controllers

