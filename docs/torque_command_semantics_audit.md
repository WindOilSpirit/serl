# ROS2 SERL Torque Command Semantics Audit

日期：2026-06-24

## 目标

本轮排查不再优先怀疑 Jacobian Eigen::Map、O_T_EE Map、q/dq 来源或 reference limiting。重点验证 ROS2 effort command 写入语义、Coriolis/gravity 补偿、torque rate saturation、nullspace torque 是否抵消 task torque，以及底层 Franka FCI/hardware 是否会限制或中断小力矩命令。

## 官方 franka_ros2 effort 语义

本机官方/硬件源码位置：

- `/home/admin123/ros2_ws/src/franka_example_controllers/src/joint_impedance_example_controller.cpp`
- `/home/admin123/ros2_ws/src/franka_example_controllers/src/joint_impedance_with_ik_example_controller.cpp`
- `/home/admin123/ros2_ws/src/franka_example_controllers/src/gravity_compensation_example_controller.cpp`
- `/home/admin123/ros2_ws/src/franka_hardware/src/franka_hardware_interface.cpp`
- `/home/admin123/ros2_ws/src/franka_hardware/src/robot.cpp`
- `/home/admin123/ros2_ws/src/franka_hardware/include/franka_hardware/robot.hpp`

结论：

- `fr3_joint1/effort` ... `fr3_joint7/effort` 是 7 个 joint torque command interfaces。
- `FrankaHardwareInterface::write()` 在 effort controller running 时调用 `robot_->writeOnce(hw_effort_commands_)`。
- `Robot::writeOnce()` 在 effort interface active 时进入 `writeOnceJointEfforts()`。
- `writeOnceJointEfforts()` 把 7 维 effort command 包装成 `franka::Torques(efforts)`，再调用 `active_control_->writeOnce(torque_command)`。
- franka_hardware 层如果启用 `torque_command_rate_limiter_active_`，还会用 `franka::limitRate(franka::kMaxTorqueRate, torque_command.tau_J, current_state_.tau_J_d)` 限制 torque rate。
- `robot.hpp` 注释说明 joint effort command 是每个关节的 torque command。

因此 ROS2 command interface 写入的 effort 语义是 libfranka joint torque command，不是 joint position/velocity，也不是 Cartesian wrench。

## 官方 example 公式

### JointImpedanceExampleController

请求 7 个 effort command interfaces：

```cpp
arm_id_ + "_joint" + std::to_string(i) + "/effort"
```

控制律：

```cpp
tau_d_calculated =
    k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);
command_interfaces_[i].set_value(tau_d_calculated(i));
```

该 example 不显式添加 Coriolis。

### JointImpedanceWithIKExampleController

控制律：

```cpp
coriolis = franka_robot_model_->getCoriolisForceVector();
q_error = joint_positions_desired - joint_positions_current;
tau_d_calculated =
    k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_) + coriolis;
command_interfaces_[i].set_value(tau_d_calculated(i));
```

该 example 添加 Coriolis，不显式添加 gravity。

### GravityCompensationExampleController

控制律：

```cpp
command_interface.set_value(0);
```

该 example 只写零 torque。结合 libfranka torque-control 语义，显式 gravity torque 不应由 controller 自己再加一遍。

## 当前 SERL ROS2 controller 公式

当前文件：

- `serl_franka_ros2_control/src/serl_cartesian_impedance_controller.cpp`

当前核心公式：

```cpp
wrench = -K * cartesian_error - D * cartesian_velocity - Ki * error_i;
tau_task = jacobian.transpose() * wrench;
tau_nullspace_enabled = enable_nullspace_torque ? tau_nullspace : 0;
tau_d_calculated = tau_task + tau_nullspace_enabled + coriolis;
tau_d_saturated = saturate_torque_rate(tau_d_calculated, robot_state->tau_J_d);
command_interfaces_[i].set_value(tau_d_saturated(i));
```

对齐情况：

- Coriolis：SERL ROS2 当前添加 Coriolis；这与官方 `JointImpedanceWithIKExampleController` 一致。
- Gravity：SERL ROS2 当前不显式添加 gravity；这与官方 examples 和 libfranka torque-control 语义一致。
- Command interface：SERL ROS2 写入的也是 7 个 joint effort command interfaces，与官方 examples 同一接口语义。
- Torque rate saturation previous torque：SERL ROS2 使用 `robot_state->tau_J_d`。franka_hardware 层自身也用 `current_state_.tau_J_d` 做 `franka::limitRate` 的参考。
- `torque_rate_limit` 单位：当前 SERL controller 的 `torque_rate_limit` 是每个 controller update cycle 的 Nm 差值限制；franka_hardware/libfranka 的 `kMaxTorqueRate` 也是对相邻周期 torque command 的限制。

本轮修正：

- 新增参数 `enable_nullspace_torque`，默认 `false`。
- 实际 torque command 变为 `tau_task + coriolis`，除非显式开启 nullspace torque。
- debug/status 新增：
  - `tau_nullspace_1..7`
  - `dot_tau_task_tau_nullspace`
  - `desired_wrench_x/y/z`
  - `desired_wrench_torque_x/y/z`
  - `wrench_est_x/y/z`
  - `wrench_est_torque_x/y/z`
  - `wrench_est_error_*`
  - `wrench_est_error_norm`

## 官方 example 对照结果

### 直接运行官方 example.launch.py

命令使用 `franka_bringup example.launch.py controller_name:=joint_impedance_example_controller`。该路径未能启动 controller_manager：

```text
symbol lookup error:
/home/admin123/ros2_ws/install/franka_hardware/lib/libfranka_hardware.so:
undefined symbol: fmt::v12::vformat...
```

同时该 launch 读取 `/home/admin123/ros2_ws/src/franka_bringup/config/franka.config.yaml`，namespace 为 `/NS_1`，robot_ip 为 `172.16.0.3`，与当前 teleop_test 工作路径使用的 `/controller_manager` 和默认 robot_ip 不同。

因此本轮不采用该 launch 路径作为对照结果。

### 同一工作 bringup 路径启动官方 GravityCompensationExampleController

使用当前 teleop_test 已验证可工作的 bringup/spawner 路径，controller 成功 load/configure/activate：

```text
gravity_compensation_example_controller
franka_example_controllers/GravityCompensationExampleController
active
```

`ros2 control list_hardware_interfaces` 显示：

- `fr3_joint1/effort` ... `fr3_joint7/effort` 均为 available + claimed。

记录文件：

- `Code/spacemouse_franka_teleop_test/debug_output/official_gravity_compensation_joint_states.csv`

3 秒内 `/franka/joint_states` 关节位置范围：

| joint | q range (rad) |
|---|---:|
| 1 | 1.259e-05 |
| 2 | 1.675e-05 |
| 3 | 1.366e-05 |
| 4 | 1.366e-05 |
| 5 | 1.127e-05 |
| 6 | 8.862e-06 |
| 7 | 1.338e-05 |

解释：

- 官方 zero-effort gravity compensation controller 可以 active 并 claim effort interfaces。
- 该 controller 只写 0 torque，因此不应期待明显主动运动。
- 本次官方 moving torque example 没有继续运行：一是直接官方 launch 路径存在 fmt ABI/namespace/ip 问题；二是官方 `JointImpedanceExampleController` 内置轨迹不是专门的极小位移验证轨迹；三是最小 joint sine effort 测试已经触发过 Franka communication reflex，不适合继续增加运动风险。

## 最小 joint-level torque 验证结果

新增 controller：

- `serl_franka_ros2_control/JointSineTorqueTestController`
- 源码：`serl_franka_ros2_control/src/joint_sine_torque_test_controller.cpp`
- 配置：`serl_franka_ros2_control/config/joint_sine_torque_test_controller.yaml`
- 记录脚本：`Code/spacemouse_franka_teleop_test/scripts/debug/test_joint_sine_torque.py`

默认测试命令：

```text
tau_4 = 0.05 * sin(2*pi*0.2*t) Nm
start_delay_s = 2.0
duration_s = 3.0
其他关节 torque = 0
```

安全限制：

- `amplitude_nm <= 0.2`
- `frequency_hz <= 1.0`
- `duration_s <= 5.0`

实际结果：

- controller 成功 load/configure/activate。
- claimed command interfaces 顺序：
  - `fr3_joint1/effort`
  - ...
  - `fr3_joint7/effort`
- 未能完成 CSV 记录。
- ros2_control_node 退出，Franka 报：

```text
franka::ControlException:
libfranka: Move command aborted: motion aborted by reflex!
["communication_constraints_violation"]
```

解释：

- 这个结果没有证明 `tau_4=0.05 Nm` 会或不会带来关节响应，因为测试被底层 communication reflex 中断。
- 它说明当前环境下“直接自定义 effort controller 做小关节力矩验证”还受到 FCI/实时通信稳定性的限制。
- 这比 Cartesian/Jacobian 问题更底层，应优先检查 network/FCI/RT scheduling/CPU isolation/franka_hardware timing。

## Nullspace-off fixed +1 mm x 结果

记录文件：

- `Code/spacemouse_franka_teleop_test/debug_output/fixed_offset_k2000_nullspace_off.csv`

测试条件：

- `translational_stiffness = 2000`
- `enable_nullspace_torque = false`
- `reference_limit_mode = per_axis_error_clip`
- `use_robot_state_q_dq = true`
- `reference_clipped = 0`

结果：

| metric | value |
|---|---:|
| offset phase rows | 244 |
| measured_x delta | 0.031860 mm |
| final target - measured | 0.969036 mm |
| mean cartesian_force_x | 1.861517 N |
| max cartesian_force_x | 2.007620 N |
| mean commanded_torque_norm | 0.947215 Nm |
| max commanded_torque_norm | 1.117240 Nm |
| wrench_est_error_norm | 0.0 |
| enable_nullspace_torque | 0 |

与之前 K=2000、nullspace 未关闭时约 `0.046576 mm` 的 x 方向响应相比，关闭 nullspace 后没有改善。因此 nullspace torque 抵消 task torque 不是当前主因。

## Task torque 到 wrench 一致性

在 fixed offset CSV 中：

- `wrench_est = pinv(J.transpose()) * tau_task`
- `wrench_est_x` 与 `cartesian_force_x` 完全一致。
- `wrench_est_error_norm = 0.0`。

这说明当前 `tau_task = J^T * wrench` 生成的 task torque 在线性代数反推上确实对应期望 wrench，至少本次 +x 测试没有显示 task torque 构造方向错误或被冗余空间主导。

## 当前判断

已经进一步排除：

- target 没发；
- controller 没 active；
- effort interfaces 没 claimed；
- reference limiting 卡住；
- Jacobian/O_T_EE/q-dq 简单映射问题；
- task torque 对 desired wrench 的线性代数映射错误；
- nullspace torque 抵消 task torque。

仍然最可疑：

- Franka FCI / realtime communication 稳定性；
- franka_hardware effort control loop timing；
- 底层 torque control safety/reflex 对小 torque controller 的中断；
- 当前控制 PC 网络/CPU/RT kernel 配置；
- 当前 robot mode/error recovery 状态；
- Cartesian impedance 参数产生的 torque 太小，不足以克服真实系统静摩擦/内部阻尼/末端环境，但这需要在底层 effort path 稳定后再判定。

## 后续建议

1. 优先修复或确认 `communication_constraints_violation`：
   - 检查 FCI 网口、交换机/直连、EEE/offload、CPU governor、RT priority、CPU isolation。
   - 复查 franka_hardware 日志中的 update/read/write timing。
2. 等 FCI 通信稳定后，重跑 `JointSineTorqueTestController`：
   - 先 `amplitude_nm=0.02`，duration 2 s；
   - 再 `0.05 Nm`；
   - 记录 q4/dq4/tau4。
3. 若 joint sine effort path 能稳定响应，再回到 SERL fixed +1 mm x：
   - 保持 `enable_nullspace_torque=false`；
   - 比较 `tau_task + coriolis` 与官方 moving torque example 的响应尺度。
4. 不建议在 communication reflex 未解决前继续增大 Cartesian stiffness 或运行 SpaceMouse teleop。
