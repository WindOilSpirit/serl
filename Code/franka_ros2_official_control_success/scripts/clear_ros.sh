#!/usr/bin/env bash
set -eo pipefail

pkill -INT -f "ros2 launch franka_bringup" || true
pkill -INT -f "ros2_control_node" || true
pkill -INT -f "controller_manager" || true
pkill -INT -f "gravity_compensation_example_controller" || true
pkill -INT -f "joint_position_example_controller" || true
sleep 3

pgrep -af "franka.launch.py|ros2_control_node|controller_manager|gravity_compensation_example_controller|joint_position_example_controller" || true

