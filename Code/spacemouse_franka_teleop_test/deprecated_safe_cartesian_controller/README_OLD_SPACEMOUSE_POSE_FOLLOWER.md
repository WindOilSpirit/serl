# SpaceMouse Franka Teleop Test

这是一个独立的 SpaceMouse -> Franka teleop 测试包，不包含 SERL 训练逻辑。底层仍使用 SERL 的 `serl_safe_cartesian_pose_controller`，SpaceMouse 不直接向底层 Cartesian controller 发布 target pose。

## 控制链路

```text
SpaceMouse
-> spacemouse_franka_teleop_test
-> /spacemouse_franka_teleop/action        # TwistStamped, 表示归一化加速度意图
-> /spacemouse_franka_teleop/deadman       # Bool, 只表示 deadman 是否按下
-> /spacemouse_franka_teleop/retreat       # Bool, 只表示 retreat 按键请求
-> /spacemouse_franka_teleop/fine_mode     # coarse / fine
-> spacemouse_franka_pose_action_server
-> measured current_pose 同步 / deadzone 归一化 / power scaling
-> flight-style acceleration shaping / damping / jerk limiting
-> velocity limiting / per-step delta limiting / workspace slow-down
-> tracking-error guard / force guard / joint-margin guard
-> /serl_safe_cartesian_pose_controller/target_pose
-> SERL Cartesian pose controller
-> Franka cartesian_pose_command
```

第一版只开放 x/y/z 平移。姿态固定为当前机器人姿态，不控制 roll/pitch/yaw。

## 真机控制命令

Franka 真机链路默认使用系统已经隔离的 CPU2。启动脚本会在每次启动前检查 CPU2 governor、最低频率、进程绑定和 CPU 隔离状态：CPU2 必须是 `performance` 模式，且最低频率必须不低于 `2000000 kHz`。如果 CPU2 没有系统级隔离，临时调试可设置 `FRANKA_RT_REQUIRE_ISOLATED=0`，但这不等价于真正的 CPU 专用。

推荐每条启动命令单独开一个终端，按顺序执行。

终端 0：首次运行或修改代码后，构建 SpaceMouse teleop overlay。

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/build_overlay.sh
```

终端 1：清理旧 ROS/Franka 进程。

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/clear_ros.sh
```

终端 2：启动 Franka bringup。

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/start_bringup.sh
```

终端 3：启动 SERL safe Cartesian pose controller。

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/start_serl_cartesian_controller.sh
```

终端 4：启动 SpaceMouse teleop 和 robot server。

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/run_teleop.sh
```

终端 5：检查 CPU2 和进程运行状态。

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/check_cpu30_runtime.sh
```

### 可视化控制 UI

也可以启动本地 dashboard，用按钮执行 clear、bringup、controller、teleop、target reset 和数据记录，并实时查看 SpaceMouse 指令、Franka 当前绝对位姿、当前 target pose、server 状态与 discontinuity 告警。
左侧 `Safety Gates` 面板会实时显示 current pose fresh、robot state fresh、target gate、bottom controller target enable、publish block reason、底层 controller target acceptance，以及 `controller_internal_command_pose` 与 measured/target 的连续性误差。

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/build_overlay.sh
./scripts/run_dashboard.sh
```

UI 按钮对应的后台命令：

- `Clear`: `./scripts/clear_ros.sh`，同时清理所有残留 teleop launch、teleop node、pose action server 进程
- `Bringup`: `./scripts/start_bringup.sh`
- `Controller`: `./scripts/start_serl_cartesian_controller.sh`
- `Teleop`: `./scripts/run_teleop.sh`
- `Reset Target`: 调用 `/spacemouse_franka_pose_action_server/reset_to_current_pose`，只有 measured pose fresh 时才会把 `server_target_pose / hold_pose / previous_published_target` 同步到 `measured_pose_from_franka`，并清零 `v_cmd/a_cmd/raw_action`。如果 measured pose stale，会明确 rejected
- `Start Recording`: 以 1000 Hz 采样当前 UI 已收到的最新状态，先写入临时 CSV
- `Stop Recording`: 停止记录，弹出 CSV 文件命名与保存窗口

UI 订阅的话题：

- SpaceMouse 指令：`/spacemouse_franka_teleop/action`
- deadman 输入事实：`/spacemouse_franka_teleop/deadman`
- retreat 输入事实：`/spacemouse_franka_teleop/retreat`
- fine mode：`/spacemouse_franka_teleop/fine_mode`
- Franka 当前绝对位姿：`/franka_robot_state_broadcaster/current_pose`
- Franka 上一命令/期望位姿：`/franka_robot_state_broadcaster/last_desired_pose`
- 当前目标位姿：`/serl_safe_cartesian_pose_controller/target_pose`
- server 状态和 discontinuity 告警：`/spacemouse_franka_teleop/server_status`

UI 调用的 service：

- target 对齐复位：`/spacemouse_franka_pose_action_server/reset_to_current_pose`

UI 记录 CSV 字段包括：

- `frame`, `time_unix_s`, `time_since_start_s`
- `machine_state`: 1 表示 bringup 正在运行或当前位姿仍新鲜；0 表示 bringup 已异常退出或没有可用机器人状态
- `bringup_running`, `bringup_exit_code`, `controller_running`, `controller_exit_code`, `teleop_running`, `teleop_exit_code`
- `teleop_state`, `fine_mode`
- `spacemouse_linear_x/y/z`, `spacemouse_angular_x/y/z`
- `current_x/y/z/qx/qy/qz/qw`, `target_x/y/z/qx/qy/qz/qw`
- `target_minus_current_x/y/z`
- `server_status`, `alert`

如果 CPU2 尚未隔离但需要继续真机调试：

```bash
FRANKA_RT_REQUIRE_ISOLATED=0 ./scripts/start_bringup.sh
FRANKA_RT_REQUIRE_ISOLATED=0 ./scripts/start_serl_cartesian_controller.sh
FRANKA_RT_REQUIRE_ISOLATED=0 ./scripts/run_teleop.sh
```

`start_serl_cartesian_controller.sh` 启动的 controller 应为：

```text
serl_safe_cartesian_pose_controller
serl_franka_ros2_control/SafeCartesianPoseController
```

## 操作方式

- teleop node 只发布 raw action、deadman、retreat、fine mode，不直接发布底层 controller target。
- 按住 deadman 后，robot server 检查 controller active、measured pose fresh、controller internal command fresh、command topic connected、publisher count 正常，并确认 `controller_internal_command_pose - measured_pose` 小于 `tracking_start_tolerance`，才会进入 `HUMAN_CONTROL`。
- 松开 deadman 后不会直接把 `v_cmd` 置零，而是进入 jerk-limited brake。
- 同时按住 fine 按键会切换到 fine 限制。
- `IDLE` 中不发布 target；底层 controller 自己保持 continuous internal command pose。

## 当前控制逻辑

启动或 controller activate 时：

- server 读取 `/franka_robot_state_broadcaster/current_pose`
- server 同时读取 `/franka_robot_state_broadcaster/last_desired_pose` 作为诊断字段
- server 只使用 `/franka_robot_state_broadcaster/current_pose` 作为 measured pose；measured pose 只用于 freshness 和安全检查
- server 读取 `/serl_safe_cartesian_pose_controller/debug/internal_command_pose` 作为底层连续性参考
- `TargetManager.request_target()` 是唯一 target 写入口
- controller activate 不发布 target；manual reset 只清 target，并让底层 controller hold continuous internal command pose
- deadman 上升沿第一帧 target 必须等于 fresh `controller_internal_command_pose`
- 普通运动 target 写入仍相对上一条 target 做单周期 delta 限制
- 姿态固定为当前机器人姿态

deadman 按下时：

- raw input 先经过 `deadzone = 0.04`
- deadzone 后重新归一化：`u = sign(raw) * (abs(raw) - deadzone) / (1 - deadzone)`
- 使用可配置 power scaling：`u_scaled = sign(u) * |u|^input_power`，当前 `input_power = 2.0`
- `a_user = a_max * u_scaled`
- `a_des = a_user - d_move * v_cmd`
- 对 `a_des` 做 jerk limiting
- `v_cmd += a_cmd * dt`
- 对 `v_cmd` 做速度限幅
- `target_pose.position += v_cmd * dt`

deadman 松开时：

- `a_user = 0`
- `a_des = -d_stop * v_cmd`
- 继续 jerk-limited brake，不直接清零 `v_cmd`

每周期安全处理：

- `dt > dt_max` 时使用 `dt_nominal` 积分
- 限制单周期 `delta_pose`
- workspace 当前最小版本不做 slow-down zone；若下一步越界，则清零该轴 delta/速度/加速度
- tracking error、joint margin、force/torque guard 当前只记录日志，不切换 latch 状态
- `target_pose` 只允许由 `pose_action_server` 的主 control loop 写入；callback 只更新输入/状态标志
- IDLE 中不发布 target，也不每周期执行 `target_pose = current_pose`
- reset 不重锚定 target 到 measured pose，只清 target stream
- tracking guard 必须基于 `measured_pose_from_franka`，不使用 command pose 作为 tracking reference
- 所有 target 写入都必须经过 `TargetManager.request_target(candidate, reason, writer)`；普通运动写入若相对 previous target 的单周期差值超过 `delta_*_max`，该 target 不发布，server 回到 `IDLE`
- `robot_state_fresh=false` 会停止积分、清 target stream，并进入 `WAITING_FOR_MEASURED`
- 底层 `serl_safe_cartesian_pose_controller` 在 target stale/no target/hold service 触发时保持 continuous internal command pose；服务名 `hold_current` 保留，但语义不是强制同步到 measured pose

## Target 跳变排查

启动 teleop 前确认 command topic 没有旧 publisher：

```bash
source /home/admin123/WenshuoZhou/SERL/serl-main/scripts/source_ros2_franka_env.sh
source /tmp/spacemouse_franka_teleop_ws/install/setup.bash
ros2 topic info /serl_safe_cartesian_pose_controller/target_pose -v
```

正常启动前应为 `Publisher count: 0`；启动后应只有 `spacemouse_franka_pose_action_server` 一个 publisher。

底层 controller 还会以 10 Hz 发布调试位姿：

```text
/serl_safe_cartesian_pose_controller/debug/received_target_pose
/serl_safe_cartesian_pose_controller/debug/accepted_target_pose
/serl_safe_cartesian_pose_controller/debug/rt_target_pose
/serl_safe_cartesian_pose_controller/debug/internal_command_pose
```

用这三项区分：

- `published_target`: robot server 实际发布给 controller 的 target
- `controller_received_raw_target`: controller callback 收到的 raw target，可能随后被拒绝
- `controller_received_target`: controller 已接受并写入 target buffer 的 target
- `controller_rt_target`: controller update loop 当前缓存的 target
- `controller_internal_command`: controller 当前写入 Franka command interface 的 internal command pose
- `measured_current`: Franka measured current pose

高频 target trace 默认写入：

```text
/tmp/spacemouse_franka_target_trace.csv
```

该 CSV 包含 `measured_*`、`command_pose_*`、`controller_internal_command_*`、`internal_target_*`、`pre_guard_target_*`、`post_guard_target_*`、`previous_published_target_*`、`candidate_target_*`、`published_target_*`、`target_minus_controller_internal_command_*`、`controller_internal_command_minus_measured_*`、`target_write_reason`、`target_writer`、`final_delta_clamped`、`target_jump_rejected_reason`、`tracking_ref_type`、`robot_state_fresh`、`measured_pose_source`、`measured_pose_topic`、`measured_pose_timestamp`、`measured_pose_age_s`、`controller_internal_command_pose_age_s`、`tracking_start_tolerance`、`target_topic_publisher_count`、`target_topic_publisher_count_ok` 和 `target_initialized_from_controller_command_pose`。如果无输入时 target 跳变，先看 `target_write_reason`：

- `FIRST_HUMAN_TARGET_INTERNAL_COMMAND`: deadman 上升沿第一帧 target，等于底层 internal command pose
- `HUMAN_INTEGRATION`: 主循环积分产生 target
- `BRAKE_INTEGRATION`: deadman 松开后的平滑刹停积分

真机启动时 `target_seed_source` 正常应显示 `controller_internal_command_pose`，`tracking_ref_type` 应显示 `measured_pose_from_franka`。如果 `robot_state_callback_count` 不增长、`measured_pose_timestamp` 不更新或 `measured_pose_header_age_s` 持续增大，server 会判定 `robot_state_fresh=false` 并拒绝进入控制。如果出现 `target_jump_rejected_reason`，说明某条写入路径试图超过单周期 target delta，上层不会发布该 candidate。

UI 记录 CSV 中，`target_pose_fresh=0` 时，`target_*` 会写为 `nan`，避免把几十秒前的 topic 缓存误当成当前 target。CSV 还区分：

- `controller_process_running`: dashboard 启动脚本对应的 controller 进程是否仍在
- `ros2_controller_active`: ros2_control controller 是否 active，来自 server status
- `command_topic_connected`: target topic 是否有底层 controller subscriber
- `robot_state_fresh`: Franka measured state 是否新鲜

最小测试 1：只记录 internal target，不发布到 controller。

```bash
./scripts/run_teleop.sh \
  debug_simulation_mode:=true \
  fake_current_pose_when_debug:=true \
  publish_enabled:=false \
  force_zero_motion:=true
```

最小测试 2：真机链路中临时强制零运动，但仍发布固定 target。

```bash
./scripts/run_teleop.sh force_zero_motion:=true
```

## 初始参数

Franka 官方笛卡尔平移限制为 3.0 m/s、9.0 m/s^2、4500 m/s^3。本包 SpaceMouse 参数远低于官方限制。

SpaceMouse 中间整形参数统一放在：

```text
config/spacemouse_control_params.yaml
```

其中 `control_scale` 只缩放速度、加速度和 jerk 上限；不再缩放阻尼系数 `d_move`、刹停/急停系数 `d_stop` 和单周期 `delta_pose` 限制。`speed_scale` 保留为旧命令行覆盖入口；实际生效缩放为：

```text
effective_scale = control_scale * speed_scale
```

coarse mode:

- `control_scale = 1.0`
- `translation_deadzone = 0.04`
- `input_power = 2.0`
- `v_xy_max = 0.005 m/s`
- `v_z_up_max = 0.003 m/s`
- `v_z_down_max = 0.001 m/s`
- `a_xy_max = 0.008 m/s^2`
- `a_z_up_max = 0.006 m/s^2`
- `a_z_down_max = 0.002 m/s^2`
- `j_xy_max = 0.050 m/s^3`
- `j_z_up_max = 0.030 m/s^3`
- `j_z_down_max = 0.020 m/s^3`

brake/dt/delta/tracking:

- `d_move = 1.5 s^-1`
- `d_stop = 3.0 s^-1`
- `dt_nominal = 0.001 s`
- `dt_max = 0.003 s`
- `dx, dy <= 0.000010 m`
- `z_up <= 0.000010 m`
- `z_down <= 0.000003 m`
- `xy_error <= 0.002 m`
- `z_error <= 0.001 m`
- `joint_position_margin = 0.2 rad`

如需临时整体降低速度/加速度/jerk 上限，可以用 `speed_scale` 启动参数覆盖：

```bash
./scripts/run_teleop.sh speed_scale:=0.5
```

## 离线调试

### 独立 SpaceMouse 模拟测试

这个测试与真机控制脚本完全独立：不 source ROS overlay、不启动 controller、不向 Franka 发布任何命令。测试会读取真实 SpaceMouse，运行 10 秒，按 1000 Hz 输出 10000 帧 7 关节目标角和中间状态。

运行测试时可以操作 SpaceMouse。默认 deadman 为按钮 0，fine 为按钮 1。

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/sim_joint_targets.sh
```

输出文件在本目录：

```text
sim_outputs/joint_targets_10s.csv
```

如果需要做零输入对照，不读取 SpaceMouse：

```bash
./scripts/sim_joint_targets.sh --no-spacemouse --no-realtime
```

如果需要强制 deadman 以测试固定输入：

```bash
./scripts/sim_joint_targets.sh --no-spacemouse --force-deadman --action 0.2 0.0 0.0
```

不连真机时可用 fake current pose 跑完整软件链路：

```bash
./scripts/run_teleop.sh \
  debug_simulation_mode:=true \
  fake_current_pose_when_debug:=true \
  debug_force_deadman_after_sec:=5.0 \
  debug_z_input_after_deadman_sec:=1.0 \
  debug_z_input_value:=0.20 \
  debug_z_input_duration_sec:=3.0 \
  debug_z_input_ramp_s:=0.5
```

上下往复测试：

```bash
./scripts/run_teleop.sh \
  debug_simulation_mode:=true \
  fake_current_pose_when_debug:=true \
  debug_force_deadman_after_sec:=5.0 \
  debug_z_oscillation_mode:=true \
  debug_z_cycle_period_s:=6.0 \
  debug_z_cycle_delay_s:=0.5 \
  debug_z_cycle_value:=0.15 \
  debug_z_input_duration_sec:=12.0
```

## 注意事项

- 真机模式下必须能收到 `/franka_robot_state_broadcaster/current_pose`
- 真机模式下建议同时收到 `/franka_robot_state_broadcaster/last_desired_pose`，但它只作为诊断字段，不参与 tracking guard
- 真机模式下 joint margin guard 默认订阅 `franka/joint_states`
- teleop 节点只发布 SpaceMouse action/state/mode，不发布底层 target pose
- pose action server 是唯一生成 `/serl_safe_cartesian_pose_controller/target_pose` 的节点
- 按下 deadman 后先有 `command_start_hold_s = 0.4 s` 的零 action hold
- fine 按键只切换 server 内部限制，不在 teleop 节点提前缩放 action
- 如果仍触发 discontinuity，优先降低 `speed_scale`，其次降低 coarse/fine 的 acceleration 与 jerk 参数

## 验收项

- IDLE 中只 hold 已锁存的 target，不发送 zero pose
- IDLE 中不发布 target
- deadman 上升沿第一帧 bottom target 等于 `controller_internal_command_pose`
- deadman 松开后速度通过 `d_stop` 和 jerk limit 收敛，不直接置零
- 单周期 delta pose 不超过配置上限
- workspace 越界方向清零对应轴速度/加速度
- SpaceMouse 全输入范围内：server 输出均经过 deadzone 归一化、power scaling、damping、jerk limiting、velocity limiting、delta limiting 和 workspace final guard
