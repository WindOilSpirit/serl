# Franka SpaceMouse 当前最小控制逻辑伪代码

本文件描述当前代码路径。当前版本删除 `ARMING`、`RETREAT`、所有 latch、sync ramp、IDLE hold target 发布和 target-measured 对齐补丁。

核心语义：

```text
measured_pose_from_franka:
  真实 Franka measured pose。
  只用于 state freshness、安全检查、command-measured 误差监控。

controller_internal_command_pose:
  SafeCartesianPoseController 内部连续 command_position_。
  这是底层 controller 真正的连续运动起点。

server_target_pose:
  pose_action_server 发布给底层 controller 的 target。
  deadman 上升沿第一帧必须等于 controller_internal_command_pose。
```

## 状态

```text
ControlState:
  WAITING_FOR_MEASURED
  WAITING_FOR_CONTROLLER
  IDLE
  HUMAN_CONTROL
  BRAKE
```

## Deadman 上升沿

```text
on_deadman_rising():
  raw_action = 0
  motion_shaper.reset()

  if measured_pose not fresh:
    state = WAITING_FOR_MEASURED
    do not publish
    return

  if controller not active:
    state = WAITING_FOR_CONTROLLER
    do not publish
    return

  if controller_internal_command_pose not fresh:
    state = IDLE
    do not publish
    return

  command_to_measured = controller_internal_command_pose - measured_pose
  if norm_xy(command_to_measured) or abs(z) > tracking_start_tolerance:
    state = IDLE
    do not publish
    return

  clear server target history
  target_pose = controller_internal_command_pose
  first_target_pose = controller_internal_command_pose
  first_target_source = controller_internal_command_pose
  target_write_reason = HUMAN_CONTROL_FIRST_TARGET
  target_initialized_from_controller_command_pose = true
  control_target_valid_for_motion = true
  bottom_controller_targets_enabled = false

  call SafeCartesianPoseController.enable_targets()
  state = HUMAN_CONTROL
  first_human_target_pending = true
```

`enable_targets()` 只打开底层 target acceptance，不应自动导致运动。服务成功回调内立即发布第一帧，减少 accept_targets=true 但 first target 尚未发布的空窗：

```text
on_enable_targets_success():
  if state in [HUMAN_CONTROL, BRAKE]
     and control_target_valid_for_motion
     and target_initialized_from_controller_command_pose
     and first_human_target_pending:
    bottom_controller_targets_enabled = true
    publish target_pose  # exactly controller_internal_command_pose
    last_publish_reason = HUMAN_CONTROL_FIRST_TARGET
    first_human_target_pending = false
```

如果 enable 成功回调没有发布成功，timer loop 仍会再次尝试，但 HUMAN_CONTROL 中 `first_human_target_pending` 会阻止 SpaceMouse 积分。

```text
timer fallback:
  if first_human_target_pending and bottom_controller_targets_enabled:
  publish target_pose  # exactly controller_internal_command_pose
  first_human_target_pending = false
```

## HUMAN_CONTROL

```text
every control tick:
  if state != HUMAN_CONTROL:
    do not publish
    return

  if deadman released:
    state = BRAKE
    return

  if measured not fresh / controller inactive / publisher count invalid:
    stop integration
    clear bottom target stream
    state = WAITING/IDLE
    do not publish
    return

  raw input -> deadzone normalization -> input_power scaling
  a_user = a_max * scaled_input
  a_des = a_user - d_move * v_cmd
  jerk limit a_cmd
  integrate v_cmd
  soft velocity saturation
  delta = v_cmd * dt
  per-step delta limit
  workspace final guard:
    if next_target crosses workspace boundary:
      clamp/zero only the offending axis delta
      zero that axis velocity and acceleration

  target_pose.position += delta
  publish target_pose
```

Force guard 和 joint margin guard 目前只写日志，不切换状态。Franka 自身安全保护作为最后保护层。

## Deadman 松开 / BRAKE

```text
on_deadman_falling():
  teleop_state = IDLE
  if motion_shaper has residual velocity/acceleration:
    state = BRAKE
  else:
    clear_target()
    state = IDLE

BRAKE tick:
  a_user = 0
  a_des = -d_stop * v_cmd
  jerk limit and integrate until stopped
  target_pose += brake_delta
  publish target_pose while braking

  if motion_shaper stopped:
    call SafeCartesianPoseController.clear_target()
    clear server target history
    bottom_controller_targets_enabled = false
    state = IDLE
    do not publish further target
```

## publish gate

```text
publish_target_if_ready():
  if target_pose missing: return
  if publish_enabled == false: return
  if robot_state_fresh == false: return
  if measured_pose_valid == false: return
  if target_initialized_from_controller_command_pose == false: return
  if control_target_valid_for_motion == false: return
  if controller_active == false: return
  if state not in [HUMAN_CONTROL, BRAKE]: return
  if bottom_controller_targets_enabled == false: return

  publish target_pose
```

IDLE 不发布 target。IDLE 只等待下一次 deadman 上升沿。底层 controller 在无 target 或 target stale 时保持自己的 continuous internal command pose。

## SafeCartesianPoseController

```text
on_activate():
  require use_state_interfaces == true
  read measured current pose
  read command interface seeded command pose
  seeded_command_to_measured = norm(seeded_command_pose - measured_pose)
  if seeded_command_to_measured > activation_hold_tolerance:
    activation fails

  # Seeded command is only a sanity check. It is not the hold target.
  command_position_ = measured_position
  command_orientation_ = measured_orientation
  activation_command_to_measured_norm = norm(command_position_ - measured_position)  # should be 0
  command_seed_source = measured_pose_after_seed_sanity_check
  setCommand(measured_orientation, measured_position)
  clear all targets
  accept_targets = false
  first_target_after_enable = true

hold_current / clear_target service:
  clear all targets
  accept_targets = false
  keep setCommand(command_orientation_, command_position_)
  measured pose is only checked for warning
```

重要：`hold_current` 服务名保留，但实际语义是 hold continuous internal command pose，不是把 command 强制同步到 measured pose。由于 on_activate 已经把 command 初始化到 measured pose，后续 hold continuous command 不应再保持一个旧 seeded command 偏差。

```text
enable_targets service:
  clear all targets and hold continuous internal command pose
  accept_targets = true
  first_target_after_enable = true

target_callback(target):
  if accept_targets == false:
    reject; do not update target_position / rt_target

  if first_target_after_enable:
    target_to_command = norm(target - command_position_)
    command_to_measured = norm(command_position_ - measured_position)

    if target_to_command > first_target_tolerance:
      reject first_target_too_far_from_command

    if command_to_measured > command_measured_tracking_tolerance:
      reject command_too_far_from_measured

  accept target
  rt loop smooth-tracks from command_position_ toward accepted target
```

## 日志重点字段

```text
server_status:
  state
  robot_state_fresh
  block_reason
  tracking_block_reason
  target_jump_rejected_reason
  publish_block_reason
  target_initialized_from_controller_command_pose
  target_seed_source
  controller_internal_command_pose_age_s
  command_minus_measured_xy / z
  target_minus_command_xy / z
  tracking_start_tolerance
  control_target_valid_for_motion
  bottom_controller_targets_enabled
  controller_enable_targets_success
  target_publish_count

controller target_status:
  accept_targets
  target_accepted_count
  target_rejected_count
  last_target_reject_reason
  target_to_command_error
  target_to_measured_error
  command_to_measured_error
  controller_update_period_s
  controller_update_overrun_count
```

## 测试 CSV 调试字段

pose_action_server 的 debug CSV 现在按链路分组记录，目标是能从单个 CSV 判断“小运动”来自输入太小、整形限幅、发布阻断、底层拒收、controller 内部追踪还是 measured state stale。

```text
Input chain:
  raw_x/y/z                         # SpaceMouse 原始平移输入，轴符号前
  axis_sign_x/y/z
  input_after_axis_x/y/z            # 乘 axis_sign 后进入 MotionShaper 的输入
  translation_deadzone
  input_power
  u_deadzone_x/y/z                  # deadzone 后重新归一化，power 前
  u_scaled_x/y/z                    # power scaling + norm clip 后
  u_norm_before_clip / after_clip

Effective parameters:
  speed_scale
  fine_mode
  active_v_x/y/z_max
  active_a_x/y/z_max
  active_j_x/y/z_max
  d_move / d_stop
  delta_x/y/z_max

MotionShaper:
  a_user_x/y/z
  a_des_x/y/z
  a_cmd_x/y/z
  v_cmd_x/y/z
  pre_velocity_limit_v_x/y/z
  post_velocity_limit_v_x/y/z
  delta_x/y/z
  accel_limited_x/y/z
  jerk_limited_x/y/z
  velocity_limited_x/y/z
  delta_limited_x/y/z
  steady_v_est_x/y/z                # a_user / d_move

Server target:
  first_target_x/y/z
  first_target_source
  first_target_to_internal_command_norm
  first_target_to_measured_norm
  server_target_x/y/z
  server_target_prev_x/y/z
  server_target_delta_x/y/z
  target_write_reason
  target_writer
  target_publish_count
  last_publish_reason
  publish_block_reason

Bottom controller:
  controller_raw_received_target_x/y/z
  controller_accepted_target_x/y/z
  controller_rt_target_x/y/z
  controller_internal_command_x/y/z
  controller_accept_targets
  target_accepted_count
  target_rejected_count
  last_target_reject_reason
  target_stream_primed
  has_target
  rt_has_target

Errors:
  server_target_minus_measured_x/y/z/norm
  server_target_minus_internal_command_x/y/z/norm
  internal_command_minus_measured_x/y/z/norm
  accepted_target_minus_internal_command_x/y/z/norm
  rt_target_minus_internal_command_x/y/z/norm

Back-derived trajectory:
  target_v/a/j_x/y/z
  internal_command_v/a/j_x/y/z

Workspace:
  workspace_margin_low_x/y/z
  workspace_margin_high_x/y/z
  workspace_blocked_x/y/z
  workspace_slowdown_scale_x/y/z

Freshness / health:
  robot_state_fresh
  measured_pose_age_s
  measured_pose_update_count
  robot_state_callback_count
  ros2_controller_active
  controller_manager_available
  target_topic_publisher_count
  target_subscriber_count
  target_publish_rate_hz             # 相邻两次 publish 的实际间隔反推
  controller_update_period_s          # SafeCartesianPoseController update(period)
  controller_update_overrun_count     # update period > 3 ms 的次数
  activation_command_to_measured_norm  # activate 后 command_position_ 到 measured 的距离，验收 < 0.05~0.10 mm
  seeded_command_to_measured_norm      # sanity check 时 seeded command 到 measured 的距离
  command_seed_source                  # should be measured_pose_after_seed_sanity_check
```

dashboard 的“开始记录/停止记录”CSV 同步记录 raw received target、accepted target、rt target、internal command、controller target_status 计数和 update period。dashboard 的 1000 Hz 是用户态观测采样；controller 的 1 kHz 更新健康状态以 `controller_update_period_s` 为准。
