#!/usr/bin/env bash
set -eo pipefail

source /home/admin123/WenshuoZhou/SERL/serl-main/scripts/source_ros2_franka_env.sh

ros2 run controller_manager spawner gravity_compensation_example_controller \
  -c /controller_manager \
  -t franka_example_controllers/GravityCompensationExampleController

