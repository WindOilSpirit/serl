# Test 1：controller-only 预设 Cartesian 轨迹测试

本测试只验证控制链路的后半段：

```text
预设轨迹节点 -> SafeCartesianPoseController -> Franka -> measured pose
```

它不接入 SpaceMouse、不接入 `pose_action_server`、不使用 deadman 状态机，也不接入 `MotionShaper`、tracking backpressure、workspace guard 或 BRAKE 逻辑。

## 文件位置

```text
Code/spacemouse_franka_teleop_test/scripts/debug/test1_preset_cartesian_trajectory_node.py
Code/spacemouse_franka_teleop_test/scripts/debug/README_TEST1_PRESET_TRAJECTORY.md
```

后续新增测试也放在 `Code/spacemouse_franka_teleop_test` 下，README 使用中文，并写清楚运行命令。

## 测试目的

完整 teleop 链路通常是：

```text
SpaceMouse / pose_action_server
-> server_target_pose
-> SafeCartesianPoseController
-> Franka
-> measured_pose_from_franka
```

Test 1 只测试：

```text
preset trajectory node
-> SafeCartesianPoseController
-> Franka
-> measured_pose_from_franka
```

如果 Test 1 都不稳定，优先检查 controller 插值参数、Franka 实时性、target topic / QoS / 发布频率，以及轨迹本身是否过激。Test 1 通过后，再进入上层 shadow target 或 SpaceMouse 链路测试。

## 轨迹形式

节点只发布 Cartesian position，orientation 固定为启动时 measured orientation，并在发布前做四元数归一化与符号连续性检查。

默认轨迹主体为 x 方向复合正弦：

```text
x_ref(t) = x0 + A sin(2 pi f t) + 0.3 A sin(2 pi 3f t)
y_ref(t) = y0
z_ref(t) = z0
orientation_ref = 启动时 measured orientation
```

注意：单独使用上面的 `sin` 主体时，虽然位置从 0 连续开始，但 `t=0` 速度不是 0；如果从 `HOLD_BEFORE` 的静止状态直接切入，会造成速度阶跃，并可能触发 Franka：

```text
cartesian_motion_generator_joint_acceleration_discontinuity
```

因此当前 Test1 在复合正弦主体外乘了一个解析 quintic smooth envelope，使 motion 段起点和终点都以零速度、零加速度接入/退出。这是测试轨迹本身的一部分，不是 controller 侧的 arming、latch、alignment 或 sync ramp。

默认参数：

```text
A = 0.0005 m
f = 0.10 Hz
smooth_envelope_s = 2.0
hold_before_s = 2.0
motion_duration_s = 20.0
hold_after_s = 2.0
publish_rate_hz = 1000.0
```

启动前会解析计算参考速度、加速度、jerk，并打印：

```text
max |v_ref|
max |a_ref|
max |j_ref|
```

如果超过以下默认安全上限，节点会拒绝开始：

```text
max_v_ref_allowed = 0.005 m/s
max_a_ref_allowed = 0.020 m/s^2
max_j_ref_allowed = 0.100 m/s^3
```

## 启动前要求

运行 Test 1 前，必须确保没有其他节点向 controller target topic 发布目标：

```text
pose_action_server
spacemouse teleop node
任何手动 target publisher
```

默认目标 topic 是：

```text
/serl_safe_cartesian_pose_controller/target_pose
```

节点启动时会检查该 topic 的 publisher 数量。默认要求 publisher 数量必须等于 1，也就是只有 Test 1 节点自己。如果数量不是 1，会拒绝开始测试。

还需要确保以下 topic / service 可用：

```text
/franka_robot_state_broadcaster/current_pose
/serl_safe_cartesian_pose_controller/debug/internal_command_pose
/serl_safe_cartesian_pose_controller/debug/accepted_target_pose
/serl_safe_cartesian_pose_controller/debug/rt_target_pose
/serl_safe_cartesian_pose_controller/debug/target_status
/serl_safe_cartesian_pose_controller/enable_targets
/controller_manager/list_controllers
```

当前 controller 已有 `enable_targets` 服务门控。Test 1 默认会调用这个已有服务，但不会给 controller 增加新的 arming、latch、alignment 或 sync ramp 逻辑。

## 推荐运行流程

以下命令默认在仓库根目录运行：

```bash
cd /home/admin123/WenshuoZhou/SERL
```

### 终端 1：启动 Franka bringup

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/start_bringup.sh
```

保持这个终端打开。

### 终端 2：启动 SafeCartesianPoseController

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/start_serl_cartesian_controller.sh
```

保持这个终端打开。

### 终端 3：确认 controller 状态

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/list_controllers.sh
```

确认 `serl_safe_cartesian_pose_controller` 为 `active`。

也可以直接查看 topic publisher / subscriber：

```bash
source /home/admin123/WenshuoZhou/SERL/serl-main/scripts/source_ros2_franka_env.sh
ros2 topic info /serl_safe_cartesian_pose_controller/target_pose
ros2 topic echo --once /franka_robot_state_broadcaster/current_pose
ros2 topic echo --once /serl_safe_cartesian_pose_controller/debug/internal_command_pose
ros2 topic echo --once /serl_safe_cartesian_pose_controller/debug/target_status
```

在运行 Test 1 前，`/serl_safe_cartesian_pose_controller/target_pose` 不应有其他上层 publisher。

### 终端 4：运行 Test 1

```bash
cd /home/admin123/WenshuoZhou/SERL
source serl-main/scripts/source_ros2_franka_env.sh
python3 Code/spacemouse_franka_teleop_test/scripts/debug/test1_preset_cartesian_trajectory_node.py \
  --ros-args \
  -p axis:=x \
  -p amplitude_m:=0.0005 \
  -p frequency_hz:=0.10 \
  -p hold_before_s:=2.0 \
  -p motion_duration_s:=20.0 \
  -p hold_after_s:=2.0 \
  -p output_dir:=Code/spacemouse_franka_teleop_test/debug_output/test1_x_0p5mm_0p10hz
```

如果你只想用默认输出目录，可以省略 `output_dir`。默认会写到：

```text
debug_output/test1_<timestamp>/
```

## 后续逐级测试命令

先从 0.5 mm、0.10 Hz 通过后，再逐级增大：

```bash
# 1 mm, 0.10 Hz
python3 Code/spacemouse_franka_teleop_test/scripts/debug/test1_preset_cartesian_trajectory_node.py \
  --ros-args \
  -p axis:=x \
  -p amplitude_m:=0.0010 \
  -p frequency_hz:=0.10 \
  -p output_dir:=Code/spacemouse_franka_teleop_test/debug_output/test1_x_1p0mm_0p10hz

# 1 mm, 0.20 Hz
python3 Code/spacemouse_franka_teleop_test/scripts/debug/test1_preset_cartesian_trajectory_node.py \
  --ros-args \
  -p axis:=x \
  -p amplitude_m:=0.0010 \
  -p frequency_hz:=0.20 \
  -p output_dir:=Code/spacemouse_franka_teleop_test/debug_output/test1_x_1p0mm_0p20hz

# z 方向，0.5 mm, 0.10 Hz
python3 Code/spacemouse_franka_teleop_test/scripts/debug/test1_preset_cartesian_trajectory_node.py \
  --ros-args \
  -p axis:=z \
  -p amplitude_m:=0.0005 \
  -p frequency_hz:=0.10 \
  -p output_dir:=Code/spacemouse_franka_teleop_test/debug_output/test1_z_0p5mm_0p10hz
```

不要直接从 2 mm 或高频开始。

## 常用参数

```text
target_topic
measured_pose_topic
internal_command_topic
accepted_target_topic
rt_target_topic
controller_status_topic
controller_manager_service
enable_targets_service
controller_name
frame_id
publish_rate_hz
axis
amplitude_m
frequency_hz
third_harmonic_ratio
smooth_envelope_s
hold_before_s
motion_duration_s
hold_after_s
max_v_ref_allowed
max_a_ref_allowed
max_j_ref_allowed
max_pose_age_s
max_initial_command_measured_m
initial_settle_s
output_dir
output_csv
call_enable_targets
require_single_publisher
require_controller_subscriber
controller_wait_timeout_s
state_wait_timeout_s
```

如果 1000 Hz Python 发布抖动过大，可先降低发布频率做诊断：

```bash
-p publish_rate_hz:=500.0
```

## 预检查退出码

如果终端显示类似：

```text
The terminal process "/usr/bin/bash" terminated with exit code: 6.
```

说明 Test1 节点在真正发布轨迹前主动退出。常见退出码如下：

```text
2  参考轨迹 v/a/j 超过安全上限
3  等待 measured pose 超时
4  等待 controller internal command pose 超时
5  controller 未处于 active 状态
6  controller_internal_command_pose 与 measured_pose 初始距离过大
7  target topic 不止 Test1 一个 publisher
8  target topic 没有 subscriber
9  enable_targets 服务调用失败
```

### exit code 6 的含义

`exit code 6` 是启动前安全检查失败：

```text
internal_command_minus_measured_norm > max_initial_command_measured_m
```

默认上限是：

```text
max_initial_command_measured_m = 0.00050 m
```

也就是 0.50 mm。这个值和当前 `SafeCartesianPoseController` 参数中的 `command_measured_tracking_tolerance_m: 0.0005` 对齐。这个检查发生在轨迹发布前，所以出现 code 6 时，Test1 没有开始运动。

处理建议：

```bash
# 1. 等待 3 到 5 秒，让 controller hold 稳定后重新运行 Test1。

# 2. 确认没有其他 target publisher。
source /home/admin123/WenshuoZhou/SERL/serl-main/scripts/source_ros2_franka_env.sh
ros2 topic info /serl_safe_cartesian_pose_controller/target_pose

# 3. 查看 measured pose 和 internal command pose 是否正常刷新。
ros2 topic echo --once /franka_robot_state_broadcaster/current_pose
ros2 topic echo --once /serl_safe_cartesian_pose_controller/debug/internal_command_pose

# 4. 确认 controller 仍然 active。
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/list_controllers.sh
```

如果修改前旧版本脚本使用的是 0.10 mm 阈值，容易在 controller 实际仍可接受 first target 的情况下提前退出。当前版本已经默认改为 0.50 mm，并且会在 controller active 后等待 `initial_settle_s=2.0` 秒再检查。

如果仍然 code 6，说明 internal command 与 measured pose 已经超过 controller 自己的 first target 跟踪容忍度。此时不要继续强行放宽阈值，优先重新启动 controller 或检查真机状态。

只在临时诊断时，可以显式调整阈值观察状态：

```bash
python3 Code/spacemouse_franka_teleop_test/scripts/debug/test1_preset_cartesian_trajectory_node.py \
  --ros-args \
  -p amplitude_m:=0.0005 \
  -p frequency_hz:=0.10 \
  -p max_initial_command_measured_m:=0.0010 \
  -p output_dir:=Code/spacemouse_franka_teleop_test/debug_output/test1_diag_relaxed_initial_gap
```

正式 Test1 建议仍使用默认 `0.00050 m`，因为这个值与 controller 当前 first target 安全检查一致。

## 输出文件

测试结束后会在 `output_dir` 中生成：

```text
test1_preset_cartesian_trajectory.csv
positions.png
tracking_errors.png
reference_vaj.png
target_backderived_vaj.png
controller_internal_vaj.png
orientation_continuity.png
publish_timing.png
```

CSV 每个 publish tick 记录一行，主要包含：

```text
t, dt, phase, publish_count, publish_rate_hz_observed
x_ref, y_ref, z_ref
vx_ref, vy_ref, vz_ref
ax_ref, ay_ref, az_ref
jx_ref, jy_ref, jz_ref
target_x/y/z/qx/qy/qz/qw
target_v/a/j_backderived_x
controller_internal_command_x/y/z
controller_accepted_target_x/y/z
controller_rt_target_x/y/z
controller_accept_targets
target_accepted_count
target_rejected_count
last_target_reject_reason
controller_update_period_s
controller_update_overrun_count
measured_x/y/z/qx/qy/qz/qw
measured_pose_age_s
robot_state_fresh
measured_pose_update_count
target_minus_internal_norm
internal_minus_measured_norm
target_minus_measured_norm
target_quat_norm
measured_quat_norm
target_quat_dot_prev
orientation_error_angle_rad
target_topic_publisher_count
target_topic_subscriber_count
controller_active
controller_manager_available
franka_error_detected
error_reason
```

`controller_internal_v/a/j_backderived_x` 是从 controller debug pose 样本反推的。由于 debug pose 的发布频率受 controller 参数 `debug_publish_period_s` 限制，它适合做诊断参考，不代表 1 kHz controller 内部真实导数。

## 通过标准

初始 Test 1 通过应满足：

```text
1. 不报 Franka error。
2. robot_state_fresh 始终为 True。
3. target_accepted_count 持续增加。
4. target_rejected_count 不增加。
5. controller_update_overrun_count 不增加或极少增加。
6. target_minus_internal_norm 不快速扩大。
7. internal_minus_measured_norm 不超过约 0.3 到 0.5 mm。
8. measured_x 能跟随 controller_internal_command_x。
9. target 反推 v/a/j 与解析 v/a/j 一致，没有尖峰。
10. target_quat_dot_prev 不出现负值。
```

## 失败时如何判断

如果启动前 reference v/a/j 超限，降低 `amplitude_m` 或 `frequency_hz`。

如果 `target_minus_internal_norm` 增大，说明 controller 跟不上 target，优先检查 controller 插值参数。

如果 `internal_minus_measured_norm` 增大，说明真机没有跟上 controller internal command，优先检查 FCI 状态、实时性、负载或是否接触环境。

如果 `target_rejected_count` 增加，检查 first target 连续性、`accept_targets` 服务、target stamp、controller tolerance 和是否有其他 publisher。

如果 Franka 报：

```text
cartesian_motion_generator_joint_acceleration_discontinuity
```

说明 target 轨迹在切入、切出或局部变化上对 Franka 过激。当前 Test1 已默认启用 `smooth_envelope_s=2.0` 来避免从静止 hold 切入非零速度；如果仍然触发，先降低 `amplitude_m` / `frequency_hz`，或增大 `smooth_envelope_s`。

如果 publish `dt` 抖动明显，先降低 `publish_rate_hz`；如果问题只在 Python 1000 Hz 下出现，再考虑写 C++ 版本测试节点。
