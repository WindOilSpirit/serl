# Franka ROS2 Official Control Success Test

This folder records the clean control path that successfully validates Franka
ROS2 control using official Franka ROS2 components only.

It intentionally does not contain a custom controller. It does not depend on
`franka_circle_test`, direct `libfranka` test code, SERL, HIL-SERL, SpaceMouse,
actor/learner, reward classifier, demo collection, or replay buffer code.

## What This Test Proves

- The PC can connect to the Franka through FCI at `172.16.0.2`.
- ROS2 `franka_bringup` can start `ros2_control_node`.
- Official `franka_example_controllers` can be loaded through
  `controller_manager`.
- Official control can be activated when the robot is not in `User stopped`
  mode.

## Hardware And Environment

- Robot: Franka FR3
- Robot IP: `172.16.0.2`
- ROS: Humble
- Franka ROS2 workspace: `/home/admin123/ros2_ws`
- Project root: `/home/admin123/WenshuoZhou/SERL`
- Franka network interface observed during debugging: `eno1`

Before running motion control, confirm:

```bash
uname -a
cat /sys/kernel/realtime
ulimit -r
```

Expected:

```text
PREEMPT_RT
1
99
```

## Important Franka State Note

If controller activation fails with:

```text
Move command rejected: command not possible in the current mode ("User stopped")
```

then ROS is not the primary problem. The robot is in Franka `User stopped` mode.
Release the user stop from Desk / Pilot / robot UI, make sure FCI external
control is allowed, then restart the ROS bringup.

## Clean Start

Terminal 1:

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/franka_ros2_official_control_success
./scripts/clear_ros.sh
./scripts/start_bringup.sh
```

Leave this terminal open. It runs `franka_bringup`.

## List Controllers

Terminal 2:

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/franka_ros2_official_control_success
./scripts/list_controllers.sh
```

This confirms that official Franka example controller plugins are visible.

## Non-Motion Official Control Test

Use this first. It activates official torque control for gravity compensation
and should not command a visible trajectory.

Terminal 2:

```bash
./scripts/start_gravity_compensation.sh
```

Check state:

```bash
./scripts/list_controllers.sh
./scripts/robot_state_once.sh
```

Stop it:

```bash
./scripts/stop_controller.sh gravity_compensation_example_controller
```

## Visible Official Motion Test

Run only when the workspace is clear.

```bash
./scripts/start_joint_position.sh
```

This uses the official `JointPositionExampleController` from
`franka_example_controllers`.

Stop it:

```bash
./scripts/stop_controller.sh joint_position_example_controller
```

## Shutdown

Stop any active controller first:

```bash
./scripts/stop_controller.sh joint_position_example_controller
```

Then press `Ctrl+C` in the bringup terminal, or run:

```bash
./scripts/clear_ros.sh
```

## What Was Removed From This Folder

Earlier failed attempts included:

- a custom Cartesian pose controller under `franka_circle_test`
- direct `libfranka` motion test code
- a first draft official-controller wrapper folder

Those are intentionally not part of this cleaned success folder.

