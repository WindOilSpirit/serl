# SpaceMouse Franka Teleop Test

这个目录现在只保留新的测试路线：

```text
SpaceMouse
-> spacemouse_franka_teleop_test/teleop_node
-> /spacemouse_franka_teleop/action
-> spacemouse_franka_impedance_teleop_server
-> /serl_cartesian_impedance_controller/target_pose
-> SERL-style Cartesian impedance controller
-> Franka
```

旧的 `SafeCartesianPoseController` 路线已经移入：

```text
Code/spacemouse_franka_teleop_test/deprecated_safe_cartesian_controller/
```

旧路线只用于历史追溯，当前 teleop、launch、dashboard 和 README 都不再使用旧 controller 的 `internal_command`、`accepted_target`、`enable_targets`、`hold_current` 等内容。

## 当前接通的控制器

默认底层控制器名称：

```text
serl_cartesian_impedance_controller
```

teleop server 发布目标位姿到：

```text
/serl_cartesian_impedance_controller/target_pose
```

dashboard 会读取以下新控制器调试 topic；如果某些 topic 暂时还没有发布，界面会显示等待，不会影响 teleop server 运行：

```text
/serl_cartesian_impedance_controller/debug/raw_target_pose
/serl_cartesian_impedance_controller/debug/smoothed_target_pose
/serl_cartesian_impedance_controller/debug/clipped_target_pose
/serl_cartesian_impedance_controller/debug/measured_pose
/serl_cartesian_impedance_controller/debug/status
```

## 坐标系约定

当前新 controller 不做 TF 查询或坐标变换；它假设收到的目标位姿已经在 Franka base/world 坐标系下。代码中 debug pose 发布时统一使用 `frame_id=base`。

| 量 | frame | 依据 |
| --- | --- | --- |
| `measured_position` | Franka base/world frame，也就是 libfranka 文档中的 zero/base frame `O` | controller 从 `robot_state->O_T_EE` 取平移量；`O_T_EE` 是 end-effector 相对 base/world `O` 的位姿 |
| `target_position` | Franka base/world frame `O` | `/serl_cartesian_impedance_controller/target_pose` 的 `PoseStamped.pose.position` 被直接读入 `raw_target_`，controller 不做 TF 变换；teleop 和 fixed-offset 测试发布 `frame_id=base` |
| `limited_reference_position` | Franka base/world frame `O` | 由 `target_position` 和 `measured_position` 在同一坐标系中做 clipping/smoothing 得到 |
| `position_error` | Franka base/world frame `O` | `position_error = measured_position - limited_reference_position` |
| `cartesian_force_x/y/z` | Franka base/world frame `O` | `cartesian_force = -K * position_error - D * cartesian_velocity`；`cartesian_velocity = zeroJacobian * dq`，也在 base/world frame |
| `cartesian_torque_x/y/z` | Franka base/world frame `O` | 姿态误差按 libfranka 官方 Cartesian impedance example 转到 base/world frame 后参与 wrench |
| `wrench` | Franka base/world frame `O`，顺序为 `[Fx, Fy, Fz, Tx, Ty, Tz]` | debug 字段对应 `cartesian_force_x/y/z` 后接 `cartesian_torque_x/y/z` |
| `zeroJacobian` | Franka base/world frame `O` 下的 6x7 Jacobian，顺序为前 3 行 linear、后 3 行 angular，数组为 column-major | `franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector)`；franka_ros2/libfranka 文档说明 `zeroJacobian` 是相对 base frame 的 6x7 column-major Jacobian |

因此当前计算链路是同一坐标系内的：

```text
position_error_O = measured_position_O - limited_reference_position_O
wrench_O = [-K * position_error_O - D * (zeroJacobian_O * dq), orientation_wrench_O]
tau_task = zeroJacobian_O.transpose() * wrench_O
```

已有 Jacobian 映射校验结果显示，`column_major_FT` 能以约 `1e-7` 量级复现 controller 记录的 `tau_task`，而 row-major 或 `[T, F]` 顺序误差为 `0.5+` Nm 量级。因此当前不应对 Jacobian 做额外 transpose、reshape，也不应交换 wrench 顺序。

## 构建

在项目根目录运行：

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/build_overlay.sh
source install/setup.bash
```

## 运行前检查

本目录的脚本会 source 下面两个环境：

```bash
source /opt/ros/humble/setup.bash
source /home/admin123/ros2_ws/install/local_setup.bash
source /home/admin123/WenshuoZhou/SERL/install/local_setup.bash
```

并把 ROS 日志写到：

```text
/tmp/spacemouse_franka_teleop_ros_logs
```

先确认当前系统能看到 controller manager 和控制器：

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/list_controllers.sh
```

期望看到：

```text
franka_robot_state_broadcaster active
serl_cartesian_impedance_controller active
```

也可以检查目标 topic：

```bash
ros2 topic info /serl_cartesian_impedance_controller/target_pose -v
```

## 启动 Teleop

终端 1：启动新 teleop 链路。

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/run_teleop.sh
```

该 launch 会启动两个节点：

```text
spacemouse_franka_teleop_test
spacemouse_franka_impedance_teleop_server
```

默认按住 SpaceMouse 左键 deadman 后才会发布运动目标；同时按住右键进入 fine mode。

## 启动 UI

终端 2：启动新 dashboard。

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/run_dashboard.sh
```

UI 顶部提供三个按钮：

- `Clear`：调用 `scripts/clear_ros.sh`，清理 Franka bringup、controller manager、controller、teleop launch、teleop node 和 pose action server 相关进程；
- `Controller`：调用 `scripts/start_serl_cartesian_controller.sh`。该脚本会先启动 `franka_bringup franka.launch.py`，等待 `/controller_manager` 可用，然后 spawner `serl_cartesian_impedance_controller`；
- `Teleop`：调用 `scripts/run_teleop.sh`，只启动 SpaceMouse 节点和 pose action server。

`Controller` 按钮默认使用：

```text
FRANKA_ARM_ID=fr3
FRANKA_ROBOT_IP=172.16.0.2
SERL_CONTROLLER_MANAGER=/controller_manager
SERL_CONTROLLER_NAME=serl_cartesian_impedance_controller
```

如果新 controller 没有预先写入 `franka_bringup` 的 controller 参数，可以指定插件类型：

```bash
export SERL_CONTROLLER_TYPE='你的 ROS2 controller plugin 类型'
```

注意：当前仓库中的 `serl_franka_controllers-main` 是 ROS1/catkin `ros_control` 包，不能被 ROS2 `controller_manager` 直接加载。若 `ros2 control list_controller_types` 看不到 ROS2 版 SERL controller plugin，`Controller` 按钮会在启动 `franka_bringup` 后 spawner 失败，并在日志中提示先安装 ROS2 版 `serl_cartesian_impedance_controller`。

如果新的 controller 需要专门的启动命令，可以在启动 dashboard 前设置：

```bash
export SERL_CONTROLLER_START_CMD='你的 controller 启动命令'
Code/spacemouse_franka_teleop_test/scripts/run_dashboard.sh
```

按钮输出日志会写入：

## 运行时调参

当前开放两个运行时调试参数：

- `S_normal = normal_spacemouse_target_scale`：正常模式下 SpaceMouse 输入到 target 偏移/速度链路的放大倍数，当前默认 `2.0`；
- `S_fine = fine_spacemouse_target_scale`：微调模式下 SpaceMouse 输入到 target 偏移/速度链路的放大倍数，当前默认 `1.0`；
- `K = translational_stiffness`：SERL Cartesian impedance controller 的平移刚度，单位 `N/m`。

实际生效缩放为：

```text
normal effective scale = speed_scale * spacemouse_target_scale * normal_spacemouse_target_scale
fine effective scale   = speed_scale * spacemouse_target_scale * fine_spacemouse_target_scale
```

查看当前值：

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/show_tuning_params.sh
```

设置 SpaceMouse 放大倍数：

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/set_spacemouse_target_scale.sh 2 1
```

设置平移刚度 K：

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/set_controller_stiffness.sh 1200
```

也可以直接使用 ROS2 参数命令：

```bash
ros2 param set /spacemouse_franka_impedance_teleop_server normal_spacemouse_target_scale 2
ros2 param set /spacemouse_franka_impedance_teleop_server fine_spacemouse_target_scale 1
ros2 param set /serl_cartesian_impedance_controller translational_stiffness 1200
```

注意：S 和 K 都是运行时参数，节点已启动时设置即可生效；如果 teleop server 是旧进程，会提示参数未声明，此时请在 UI 中 `Clear` 后重新启动 `Teleop`。

```text
/tmp/spacemouse_franka_teleop_logs/
```

UI 显示内容包括：

- 新 controller 是否 active；
- SpaceMouse action、deadman、fine mode；
- teleop server 状态和 publish count；
- 发布给新 controller 的 target pose；
- Franka measured pose；
- 新 controller 的 raw target、smoothed target、clipped target、measured pose；
- reference clipping、限速/限加速度/限 jerk/限 step、commanded torque 等 status 字段；
- 外力和外力矩。

## 固定偏移测试

该测试不接入 SpaceMouse，只检查新 controller 在 x 方向 1 mm 固定目标下的响应。

终端 1：清理旧进程并启动 controller。推荐把启动进程绑定到已隔离的 CPU2：

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/clear_ros.sh
taskset -c 2 ./scripts/start_serl_cartesian_controller.sh
```

终端 2：运行固定偏移测试。

```bash
cd /home/admin123/WenshuoZhou/SERL
source Code/spacemouse_franka_teleop_test/scripts/source_ros2_franka_env.sh
Code/spacemouse_franka_teleop_test/scripts/debug/test_serl_impedance_fixed_offset.py
```

脚本流程：

```text
等待 serl_cartesian_impedance_controller active
读取 controller measured pose
发布 target = measured pose 并保持 2 s
发布 target_x = measured_x + 0.001 m 并保持 5 s
发布 target = measured pose 并保持 2 s
保存 CSV
```

默认 CSV 输出到：

```text
Code/spacemouse_franka_teleop_test/debug_output/test_serl_impedance_fixed_offset_*.csv
```

如需指定输出路径：

```bash
Code/spacemouse_franka_teleop_test/scripts/debug/test_serl_impedance_fixed_offset.py \
  --output-csv Code/spacemouse_franka_teleop_test/debug_output/my_fixed_offset.csv
```

CSV 记录字段包括：

```text
target_x
measured_x
limited_reference_x
position_error_x
cartesian_force_x/y/z
tau_task_1..7
coriolis_1..7
tau_before_saturation_1..7
tau_after_saturation_1..7
tau_command_1..7
q_1..7
dq_1..7
O_T_EE_0..15
zero_jacobian_0..41
cartesian_velocity_from_jacobian_x/y/z
measured_velocity_from_pose_diff_x/y/z
velocity_diff_norm
commanded_torque_norm
tau_rate_limited
reference_clipped
use_robot_state_q_dq
reference_limit_mode
filter_coeff
velocity_direction_cosine
velocity_norm_ratio
```

判断方式：

- `target_x` 增加 1 mm 后 `measured_x` 完全不动：controller torque 可能没有实际生效，或刚度太小；
- `measured_x` 反方向运动：控制律符号可能反了；
- `measured_x` 正方向运动但很慢、很小：刚度、阻尼或力矩尺度可能过于保守；
- `limited_reference_x` 没有接近 `target_x`：reference limiting 或 target smoothing 逻辑需要继续检查。

### 刚度对比

已提供这些 controller 参数文件：

```text
install/serl_franka_ros2_control/share/serl_franka_ros2_control/config/serl_cartesian_impedance_controller_k600.yaml
install/serl_franka_ros2_control/share/serl_franka_ros2_control/config/serl_cartesian_impedance_controller_k1200.yaml
install/serl_franka_ros2_control/share/serl_franka_ros2_control/config/serl_cartesian_impedance_controller_k2000.yaml
install/serl_franka_ros2_control/share/serl_franka_ros2_control/config/serl_cartesian_impedance_controller_k600_joint_state_qdq.yaml
install/serl_franka_ros2_control/share/serl_franka_ros2_control/config/serl_cartesian_impedance_controller_original_like.yaml
```

当前 `k600/k1200/k2000` 是安全测试配置：它们仍使用较保守的 rotational stiffness/damping 和 nullspace stiffness，但已经对齐原版的 q/dq 来源、per-axis error clip 和滤波系数：

```text
use_robot_state_q_dq: true
reference_limit_mode: per_axis_error_clip
filter_coeff: 0.005
translational_clip_*: 0.01 m
rotational_clip_*: 0.05 rad
```

`serl_cartesian_impedance_controller_k600_joint_state_qdq.yaml` 只用于 q/dq 来源对比，设置 `use_robot_state_q_dq: false`。

`serl_cartesian_impedance_controller_original_like.yaml` 更接近 ROS1 SERL 原版 dynamic_reconfigure 默认值：

```text
translational_stiffness: 2000
translational_damping: 89
rotational_stiffness: 150
rotational_damping: 7
nullspace_stiffness: 0.2
joint1_nullspace_stiffness: 100
filter_coeff: 0.005
per-axis translational clip: +/-0.01 m
per-axis rotational clip: +/-0.05 rad
torque_rate_limit: 1.0
```

注意：当前安全测试参数与 ROS1 原版 SERL 参数不同；若目标是严格复现原始 SERL 控制器行为，应优先使用 `original_like` 配置，并在真机安全边界内逐步测试。

每次测试前先清理旧进程，再用对应参数文件启动 controller。例如 1200 N/m：

```bash
cd /home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test
./scripts/clear_ros.sh
SERL_CONTROLLER_PARAM_FILE=/home/admin123/WenshuoZhou/SERL/install/serl_franka_ros2_control/share/serl_franka_ros2_control/config/serl_cartesian_impedance_controller_k1200.yaml \
  taskset -c 2 ./scripts/start_serl_cartesian_controller.sh
```

另开终端运行测试并指定输出文件：

```bash
cd /home/admin123/WenshuoZhou/SERL
source Code/spacemouse_franka_teleop_test/scripts/source_ros2_franka_env.sh
Code/spacemouse_franka_teleop_test/scripts/debug/test_serl_impedance_fixed_offset.py \
  --output-csv Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_k1200.csv
```

依次把参数文件换成 `k600`、`k1200`、`k2000`。如果刚度增大后 `cartesian_force_x` 和 `tau_before_saturation_*` 明显增大，但 `measured_x` 仍几乎不动，应优先继续查 effort 写入、硬件模式或 Jacobian/坐标映射，而不是 target 链路。

### Jacobian 映射校验

fixed offset CSV 中记录了完整 `zero_jacobian_0..41`、`cartesian_force_*`、`cartesian_torque_*` 和 `tau_task_1..7`。可以用下面脚本检查当前 Eigen 映射是否符合 libfranka/franka_ros2 的 column-major 6x7 Jacobian，以及 wrench 顺序是否为 `[Fx, Fy, Fz, Tx, Ty, Tz]`：

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/debug/verify_serl_jacobian_mapping.py \
  Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_k2000_jacobian_verify.csv
```

期望结果是 `column_major_FT` 误差最小。若 `row_major_*` 或 `column_major_TF` 更小，才说明 Jacobian reshape 或 wrench 顺序有问题。

### Cartesian velocity 一致性检查

fixed offset 脚本会在 CSV 中记录 controller 内部同一个 `update()` 周期算出的同步字段：

```text
controller_t
controller_dt
jacobian_velocity_x/y/z = zeroJacobian * dq 的平移速度前三维
jacobian_velocity_angular_x/y/z = zeroJacobian * dq 的角速度后三维
pose_diff_velocity_x/y/z = controller update 内部 O_T_EE 位置差分速度
cartesian_velocity_from_jacobian_x/y/z = jacobian_velocity_x/y/z 的兼容别名
measured_velocity_from_pose_diff_x/y/z = pose_diff_velocity_x/y/z 的兼容别名
velocity_direction_cosine = J*dq 平移速度与 pose diff 速度方向余弦
velocity_norm_ratio = |J*dq| / |pose_diff_velocity|
velocity_diff_norm = 两者差的范数
```

优先看这些 controller 内部同步字段。旧 CSV 或外部拼接 CSV 才使用下面的离线检查：

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/debug/check_serl_velocity_consistency.py \
  Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_k2000_jacobian_verify.csv
```

如果 controller 内部同步字段里的 `velocity_direction_cosine` 仍远离 1，或 `velocity_norm_ratio` 仍远离 1，才继续怀疑 Jacobian 映射、frame、`dq` 来源或硬件状态同步。注意旧 CSV 如果没有 `controller_stamp_sec`，离线脚本会 fallback 到 `time_since_start_s`，一致性判断会比新 CSV 粗糙。

本轮 q/dq 对比矩阵：

```text
1. use_robot_state_q_dq=false, K=600
2. use_robot_state_q_dq=true,  K=600
3. use_robot_state_q_dq=true,  K=1200
4. use_robot_state_q_dq=true,  K=2000
```

更完整的 Jacobian / frame / state interface 审计记录见：

```text
docs/jacobian_mapping_audit.md
```

## 主要配置

配置文件：

```text
Code/spacemouse_franka_teleop_test/config/teleop_params.yaml
```

常用字段：

```text
target_pose_topic: /serl_cartesian_impedance_controller/target_pose
controller_name: serl_cartesian_impedance_controller
require_active_controller: true
publish_enabled: true
workspace_low: [0.25, -0.20, 0.04]
workspace_high: [0.75, 0.25, 0.75]
```

如果只想在无机器人环境下检查节点启动，可临时把 `debug_simulation_mode` 和 `fake_measured_pose_when_debug` 改为 `true`，但真机测试应保持为 `false`。
