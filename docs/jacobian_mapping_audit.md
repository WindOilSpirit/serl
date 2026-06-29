# SERL ROS2 Cartesian Impedance Controller Jacobian 映射审计

本文记录当前 `serl_cartesian_impedance_controller` 中 Jacobian、`O_T_EE`、`dq`、wrench 和 joint 顺序的依据与实现状态。当前排查目标是确认 `tau = J.transpose() * wrench` 中的矩阵映射、坐标系和状态同步是否一致。

## 官方依据

`libfranka`/`franka_ros2` 对 `zeroJacobian` 的定义是：

- 返回类型：`std::array<double, 42>`
- 数学形状：`6x7`
- 存储顺序：column-major
- 坐标系：base/zero frame，也就是 Franka 文档中的 `O` frame
- 行顺序：前 3 行 linear，后 3 行 angular

本地官方文件依据：

- `/home/admin123/ros2_ws/install/libfranka/include/franka/model.h`
  - `zeroJacobian(...)` 文档写明是相对 base frame 的 `6x7` Jacobian，column-major。
- `/home/admin123/ros2_ws/src/franka_semantic_components/include/franka_semantic_components/franka_robot_model.hpp`
  - `getZeroJacobian(...)` 文档写明 `base_J_frame`，`6x7`，column-major，linear rows followed by angular rows。
- `/home/admin123/ros2_ws/src/libfranka/examples/cartesian_impedance_control.cpp`
  - 官方 impedance example 使用：
    ```cpp
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
    ```
- `serl_franka_controllers-main/src/cartesian_impedance_controller.cpp`
  - SERL 原始 ROS1 controller 同样使用：
    ```cpp
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    tau_task << jacobian.transpose() * ...
    ```

## 当前 Eigen 映射

当前 ROS2 controller 使用：

```cpp
const auto jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
```

这与 libfranka 官方 example 和 SERL ROS1 controller 保持一致。不要额外 transpose、不要 reshape 成 row-major、不要交换 wrench 线速度/角速度顺序。

已有离线 CSV 映射复现结果：

```text
column_major_FT RMS ≈ 5.5e-7
column_major_TF ≈ 0.591
row_major_TF ≈ 0.646
row_major_FT ≈ 0.890
```

解释：当前 column-major `6x7` Jacobian 和 wrench 顺序 `[Fx, Fy, Fz, Tx, Ty, Tz]` 可以复现 controller 记录的 `tau_task`。row-major 或 `[Tx, Ty, Tz, Fx, Fy, Fz]` 都明显不匹配。

## O_T_EE 解析

`libfranka` 的 `RobotState::O_T_EE` 是 measured end-effector pose in base frame，column-major `4x4`。

当前 ROS2 controller 使用：

```cpp
const Eigen::Map<const Eigen::Matrix4d> transform_matrix(robot_state->O_T_EE.data());
const Eigen::Affine3d transform(transform_matrix);
pose.position = transform.translation();
pose.orientation = Eigen::Quaterniond(transform.linear()).normalized();
```

这与官方 impedance example 的 `Eigen::Matrix4d::Map(robot_state.O_T_EE.data())` 写法一致。`O_T_EE_12/13/14` 对应 base frame 下的 measured EE position。

## q/dq 来源与 joint 顺序

控制器请求的 command interface 是 7 个 effort interface：

```text
<joint_name>/effort
```

控制器请求的 state interface 是每个 joint 的：

```text
<joint_name>/position
<joint_name>/velocity
```

之前代码在读取 `q/dq` 时假设 `state_interfaces_` 排列一定是：

```text
joint1/position, joint1/velocity, joint2/position, joint2/velocity, ...
```

现在已经改成可切换、可审计的两层检查：

1. controller 在 `on_activate()` 中按接口名解析 state interface 顺序，作为启动审计和防止 ros2_control 接口顺序假设错误；
2. impedance 控制律实际使用的 `q/dq` 由参数 `use_robot_state_q_dq` 选择。

当 `use_robot_state_q_dq: true` 时，使用同一个 Franka `RobotState`：

```cpp
Eigen::Map<const Vector7d> q(robot_state.q.data());
Eigen::Map<const Vector7d> dq(robot_state.dq.data());
q_ = q;
dq_ = dq;
```

这样 `q`、`dq`、`O_T_EE`、`tau_J_d` 和 model 计算用到的 `robot_state` 同源，更接近 libfranka 官方 Cartesian impedance example 和 SERL ROS1 controller。

当 `use_robot_state_q_dq: false` 时，使用 ros2_control joint state interfaces 中按名称解析出来的 `<joint>/position` 和 `<joint>/velocity`。这个模式只用于对比验证。

## robot_state 指针读取方式

已检查 franka_ros2 semantic component 源码：

- `/home/admin123/ros2_ws/src/franka_semantic_components/src/franka_robot_model.cpp`
- `/home/admin123/ros2_ws/src/franka_semantic_components/src/franka_robot_state.cpp`
- `/home/admin123/ros2_ws/src/franka_semantic_components/test/franka_robot_state_test.cpp`

`FrankaRobotModel::initialize()` 和 `FrankaRobotState::get_values_as_message()` 都从 `robot_state` state interface 的 `double` value 中 uncast 出 `franka::RobotState*`。`FrankaRobotState::get_robot_state()` 存在，但在当前安装版本中是 protected，不是 controller 可直接调用的 public API。

因此当前 controller 继续使用与 franka_semantic_components 内部一致的 pointer uncast 路径；没有另行修改 O_T_EE/Jacobian map。

启动时仍会打印以下接口顺序：

```text
state_interface_q_order:
  [0] fr3_joint1/position -> state_interfaces_[...]
  ...

state_interface_dq_order:
  [0] fr3_joint1/velocity -> state_interfaces_[...]
  ...

command_interface_effort_order:
  [0] fr3_joint1/effort
  ...

jacobian_columns_joint_order:
  [0] fr3_joint1
  ...
```

这些日志只在启动/激活时打印，不在 realtime `update()` 中打印。

## 坐标系定义

| 量 | 当前 frame | 说明 |
| --- | --- | --- |
| `measured_position` | base/world `O` frame | 从 `robot_state->O_T_EE` 的 translation 读取 |
| `target_position` | base/world `O` frame | `/serl_cartesian_impedance_controller/target_pose` 当前按 base frame 使用 |
| `limited_reference_position` | base/world `O` frame | target smoothing 和 reference limiting 后的参考位置 |
| `position_error` | base/world `O` frame | `measured_position - limited_reference_position` |
| `cartesian_force` | base/world `O` frame | `-K * position_error - D * cartesian_velocity` |
| `zeroJacobian` | base/world `O` frame | `getZeroJacobian(franka::Frame::kEndEffector)` |
| `wrench` | base/world `O` frame | 顺序 `[Fx, Fy, Fz, Tx, Ty, Tz]` |

因此当前控制律是：

```text
cartesian_velocity_O = zeroJacobian_O * dq
wrench_O = -K * error_O - D * cartesian_velocity_O - Ki * integral_error_O
tau_task = zeroJacobian_O.transpose() * wrench_O
tau_command = saturate_torque_rate(tau_task + tau_nullspace + coriolis, tau_J_d)
```

## Controller 内部同步 J*dq 自检

为了避免外部低频 topic 拼接导致时间不同步，controller 现在在同一个 `update()` 周期中读取并计算：

```text
q
dq
O_T_EE
zeroJacobian
cartesian_velocity_from_jacobian = J * dq
pose_diff_velocity = (measured_position_now - measured_position_prev) / dt
velocity_direction_cosine
velocity_norm_ratio = |J*dq| / |pose_diff_velocity|
velocity_diff_norm = |J*dq - pose_diff_velocity|
```

这些字段发布到：

```text
/serl_cartesian_impedance_controller/debug/status
```

新增/保留字段包括：

```text
t
dt
measured_x/y/z
pose_diff_velocity_x/y/z
measured_velocity_from_pose_diff_x/y/z
jacobian_velocity_x/y/z
jacobian_velocity_angular_x/y/z
cartesian_velocity_from_jacobian_x/y/z
dq_1..7
q_1..7
velocity_direction_cosine
velocity_norm_ratio
velocity_diff_norm
```

`test_serl_impedance_fixed_offset.py` 已经改为直接记录这些 controller 内部同步字段，不再用测试节点自己从低频 status samples 里重新差分。

## 当前结论与下一步

已确认：

- Jacobian array 的官方类型是 column-major `6x7`。
- 当前 Eigen map 与官方 example、SERL ROS1 controller 一致。
- wrench 顺序是 `[Fx, Fy, Fz, Tx, Ty, Tz]`。
- `O_T_EE` 解析方式与官方 example 一致。
- 坐标系当前均按 base/world `O` frame 对齐。
- `q/dq` 控制律来源现在由 `use_robot_state_q_dq` 参数切换；默认配置使用 `robot_state.q/dq`。
- state interface 顺序仍在启动时按名称解析并打印，用于确认 ros2_control 接口配置。

## Reference limiting 对齐

ROS1 原版不是按目标距离范数裁剪 target，而是在控制律中对 error 逐轴裁剪：

```text
error = measured - desired
error_x/y/z = clamp(error_x/y/z, -translational_clip_neg_*, translational_clip_*)
error_rx/ry/rz = clamp(error_rx/ry/rz, -rotational_clip_neg_*, rotational_clip_*)
```

ROS2 版现在新增：

```text
reference_limit_mode: per_axis_error_clip
```

这是默认模式。该模式下 controller 先计算 `measured - desired`，再按 ROS1 原版参数名逐轴 clip，clip 后的 error 进入 wrench。`limited_reference` 只用于 debug 可视化。

旧的 norm-based target clipping 保留为：

```text
reference_limit_mode: norm_target_distance
```

用于历史对比，不作为 SERL 原版复现默认模式。

## 2026-06-24 在线测试结果

测试 1：controller 内部同步 debug 已加入，但 `q/dq` 仍来自 joint state interfaces。

CSV：

```text
Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_k2000_internal_velocity.csv
```

offset 阶段结果：

```text
measured_x delta ≈ 0.0693 mm
final target-measured error ≈ 0.9274 mm
cartesian_force_x mean ≈ 1.843 N
commanded_torque_norm mean ≈ 0.903 Nm
reference_clipped = 0
torque_rate_limited = 0
controller_dt mean ≈ 0.995 ms
velocity_direction_cosine mean ≈ 0.464
velocity_norm_ratio mean ≈ 0.795
velocity_diff_norm mean ≈ 0.00220 m/s
```

测试 2：控制律 `q/dq` 改为 `robot_state.q/dq` 后，重启 controller_manager 重新加载新 shared library，再跑同一 +1 mm x fixed-offset。

CSV：

```text
Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_k2000_robot_state_qdq_velocity.csv
```

offset 阶段结果：

```text
measured_x delta ≈ 0.0498 mm
final target-measured error ≈ 0.9511 mm
cartesian_force_x mean ≈ 1.893 N
commanded_torque_norm mean ≈ 0.947 Nm
reference_clipped = 0
torque_rate_limited = 0
controller_dt mean ≈ 0.993 ms
velocity_direction_cosine mean ≈ 0.471
velocity_norm_ratio mean ≈ 1.672
velocity_diff_norm mean ≈ 0.00236 m/s
```

启动日志确认：

```text
command_interface_effort_order:
  [0] fr3_joint1/effort
  ...
  [6] fr3_joint7/effort

state_interface_q_order:
  [0] fr3_joint1/position -> state_interfaces_[0]
  ...
  [6] fr3_joint7/position -> state_interfaces_[12]

state_interface_dq_order:
  [0] fr3_joint1/velocity -> state_interfaces_[1]
  ...
  [6] fr3_joint7/velocity -> state_interfaces_[13]

jacobian_columns_joint_order:
  [0] fr3_joint1
  ...
  [6] fr3_joint7
```

`ros2 control list_hardware_interfaces` 确认 `fr3_joint1/effort` 到 `fr3_joint7/effort` 均为 available 且 claimed by active controller。

当前判断：

- effort interface claim 正常；
- target/reference limiting 正常；
- torque command 有输出且未被 torque-rate saturation 限制；
- 使用 `robot_state.q/dq` 后，同步速度自检没有明显改善；
- 因此问题仍未能归因于旧的外部 CSV 拼接不同步，也不能只归因于 joint state interface 顺序。

下一步建议检查：

```text
1. 用更大但安全的手动扰动或专门 free-drive/passive 记录，获得更高信噪比的 O_T_EE 差分速度；
2. 若同步速度自检仍异常，再做 finite-difference Jacobian：对当前 q 做小扰动正运动学差分，与 zeroJacobian 对比；
3. 检查 franka_ros2 hardware 中 robot_state 的 O_T_EE、q、dq 更新时间是否严格同一周期；
4. 检查 torque command 是否被底层 Franka safety/collision/impedance 行为软限制，虽然 effort interface 已 claimed。
```

不应优先修改 teleop、SpaceMouse 或 reference limiting。

## 2026-06-24 q/dq 参数矩阵复测

本轮修改后新增：

```text
use_robot_state_q_dq
reference_limit_mode: per_axis_error_clip
filter_coeff: 0.005
```

并用 fixed +1 mm x offset 测试了以下矩阵：

| case | use_robot_state_q_dq | K | measured_x delta | final target-measured error | mean force_x | mean torque norm | mean direction cosine | mean norm ratio | mean velocity diff |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | false | 600 | 0.003591 mm | 0.997633 mm | 0.572352 N | 0.298845 Nm | 0.516543 | 0.931315 | 0.002233 m/s |
| 2 | true | 600 | 0.002167 mm | 0.999283 mm | 0.570819 N | 0.298626 Nm | 0.442241 | 1.098724 | 0.002146 m/s |
| 3 | true | 1200 | 0.021412 mm | 0.981716 mm | 1.131087 N | 0.584094 Nm | 0.538121 | 0.943995 | 0.002240 m/s |
| 4 | true | 2000 | 0.046576 mm | 0.956392 mm | 1.848085 N | 0.934873 Nm | 0.515119 | 0.805782 | 0.002167 m/s |

CSV 文件：

```text
Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_matrix_1_qdqfalse_k600.csv
Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_matrix_2_qdqtrue_k600.csv
Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_matrix_3_qdqtrue_k1200.csv
Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_matrix_4_qdqtrue_k2000.csv
```

共同点：

```text
reference_clipped = 0
torque_rate_limited = 0
reference_limit_mode = per_axis_error_clip
filter_coeff = 0.005
limited_reference_x 到达 target_x
force_x > 0
```

结论：

- `use_robot_state_q_dq=true` 相比 `false` 没有让 K=600 的 fixed offset 跟随明显改善；
- 内部同步 `J*dq` vs pose diff 的 direction cosine 仍约 0.44-0.54，没有接近 1；
- `|J*dq| / |pose_diff|` 均值在 0.81-1.10 附近，但 direction cosine 仍差；
- stiffness 从 600 提高到 2000 时，force 和 torque 随之增大，末端 x 正向移动也增大，但 5 s 内仍只有约 0.047 mm；
- 因此当前问题不能简单归因于 q/dq state interface 与 `RobotState` 不一致。
