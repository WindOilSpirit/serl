#!/usr/bin/env bash
set -eo pipefail

pkill -INT -f "[r]os2 launch franka_bringup" || true
pkill -INT -f "[f]ranka_safe_bringup.launch.py" || true
pkill -INT -f "[r]os2_control_node" || true
pkill -INT -f "[c]ontroller_manager" || true
pkill -INT -f "[s]erl_cartesian_impedance_controller" || true
pkill -INT -f "[r]os2 launch spacemouse_franka_teleop_test" || true
pkill -INT -f "[s]pacemouse_franka_teleop.launch.py" || true
pkill -INT -f "/spacemouse_franka_teleop_test/[t]eleop_node" || true
pkill -INT -f "[t]eleop_node" || true
pkill -INT -f "spacemouse_franka_teleop_test.[t]eleop_node" || true
pkill -INT -f "[p]ose_action_server" || true
pkill -INT -f "[s]pacemouse_franka_impedance_teleop_server" || true
pkill -INT -f "spacemouse_franka_teleop_test.[p]ose_action_server" || true
sleep 2

pkill -TERM -f "[r]os2 launch franka_bringup" || true
pkill -TERM -f "[f]ranka_safe_bringup.launch.py" || true
pkill -TERM -f "[r]os2_control_node" || true
pkill -TERM -f "[c]ontroller_manager" || true
pkill -TERM -f "[s]erl_cartesian_impedance_controller" || true
pkill -TERM -f "[r]os2 launch spacemouse_franka_teleop_test" || true
pkill -TERM -f "[s]pacemouse_franka_teleop.launch.py" || true
pkill -TERM -f "/spacemouse_franka_teleop_test/[t]eleop_node" || true
pkill -TERM -f "[t]eleop_node" || true
pkill -TERM -f "spacemouse_franka_teleop_test.[t]eleop_node" || true
pkill -TERM -f "[p]ose_action_server" || true
pkill -TERM -f "[s]pacemouse_franka_impedance_teleop_server" || true
pkill -TERM -f "spacemouse_franka_teleop_test.[p]ose_action_server" || true
sleep 2

pkill -KILL -f "[r]os2_control_node" || true
pkill -KILL -f "[r]os2 launch franka_bringup" || true
pkill -KILL -f "[f]ranka.launch.py" || true
pkill -KILL -f "[s]tart_serl_cartesian_controller.sh" || true

pgrep -af "franka.launch.py|franka_safe_bringup.launch.py|ros2_control_node|controller_manager|serl_cartesian_impedance_controller|ros2 launch spacemouse_franka_teleop_test|spacemouse_franka_teleop.launch.py|teleop_node|pose_action_server|spacemouse_franka_impedance_teleop_server" || true
