#!/usr/bin/env bash
set -eo pipefail

# Standalone offline test. This does not source ROS overlays, does not launch
# controllers, and does not publish anything to Franka.
cd "$(dirname "$0")/.."
PYTHONPATH="/home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test:${PYTHONPATH:-}" \
  /home/admin123/WenshuoZhou/SERL/.venv/bin/python -B -m \
  spacemouse_franka_teleop_test.sim_joint_targets "$@"
