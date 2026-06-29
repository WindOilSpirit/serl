# ROS2 SERL Controller Plugin 审计

## 结论

当前 teleop/HIL-SERL 上层已经走 ROS2 路线，目标 controller 实例名是：

```text
serl_cartesian_impedance_controller
```

目标 topic 是：

```text
/serl_cartesian_impedance_controller/target_pose
```

原始 `serl_franka_controllers-main` 是 ROS1/catkin/ros_control controller 包，不能被 ROS2 `controller_manager` 直接加载。现在已新增顶层 ROS2 包：

```text
serl_franka_ros2_control/
```

并导出 ROS2 plugin type：

```text
serl_franka_ros2_control/SerlCartesianImpedanceController
```

## 原 ROS1 包不能直接加载的原因

- 原包使用 `catkin`、`controller_interface::MultiInterfaceController`、`controller_interface::ControllerBase`。
- 原包依赖 ROS1 `ros::NodeHandle`、`dynamic_reconfigure`、`franka_hw::FrankaModelInterface`、`franka_hw::FrankaStateInterface`、`hardware_interface::EffortJointInterface`。
- 当前系统使用 ROS2 Humble、`ament_cmake`、`ros2_control`、`controller_interface::ControllerInterface`、`pluginlib` ROS2 导出机制。
- ROS1 的 `serl_franka_controllers_plugin.xml` 不能生成 ROS2 `controller_interface__pluginlib__plugin` 索引。
- 因此 `ros2 control list_controller_types` 不会看到 ROS1 controller，`controller_manager` 也无法 load `serl_cartesian_impedance_controller`。

## 当前 ROS2 controller package

新增 package：

```text
serl_franka_ros2_control
```

关键文件：

```text
serl_franka_ros2_control/include/serl_franka_ros2_control/serl_cartesian_impedance_controller.hpp
serl_franka_ros2_control/src/serl_cartesian_impedance_controller.cpp
serl_franka_ros2_control/serl_franka_ros2_control_plugins.xml
serl_franka_ros2_control/config/serl_cartesian_impedance_controller.yaml
serl_franka_ros2_control/launch/serl_cartesian_impedance_controller.launch.py
```

依赖已包含：

```text
controller_interface
hardware_interface
pluginlib
rclcpp
rclcpp_lifecycle
geometry_msgs
franka_semantic_components
std_msgs
Eigen3
```

## Plugin 导出

`serl_franka_ros2_control_plugins.xml` 中导出：

```text
serl_franka_ros2_control/SerlCartesianImpedanceController
```

CMake 中使用：

```cmake
pluginlib_export_plugin_description_file(controller_interface serl_franka_ros2_control_plugins.xml)
```

构建并 source 后应检查：

```bash
ros2 control list_controller_types | grep -i serl
```

期望看到：

```text
serl_franka_ros2_control/SerlCartesianImpedanceController
```

## Controller manager YAML

新增配置：

```text
serl_franka_ros2_control/config/serl_cartesian_impedance_controller.yaml
```

核心条目：

```yaml
serl_cartesian_impedance_controller:
  type: serl_franka_ros2_control/SerlCartesianImpedanceController
```

controller 参数中 target topic 为：

```text
/serl_cartesian_impedance_controller/target_pose
```

## Teleop 接入

`Code/spacemouse_franka_teleop_test/config/teleop_params.yaml` 已指向新 controller：

```text
controller_name: serl_cartesian_impedance_controller
target_pose_topic: /serl_cartesian_impedance_controller/target_pose
```

dashboard 读取的新 controller debug topic：

```text
/serl_cartesian_impedance_controller/debug/raw_target_pose
/serl_cartesian_impedance_controller/debug/smoothed_target_pose
/serl_cartesian_impedance_controller/debug/clipped_target_pose
/serl_cartesian_impedance_controller/debug/measured_pose
/serl_cartesian_impedance_controller/debug/status
```

## 是否仍启动旧 SafeCartesianPoseController

当前新启动脚本：

```text
Code/spacemouse_franka_teleop_test/scripts/start_serl_cartesian_controller.sh
```

默认 spawner：

```text
serl_cartesian_impedance_controller
```

默认 type：

```text
serl_franka_ros2_control/SerlCartesianImpedanceController
```

不会启动：

```text
serl_safe_cartesian_pose_controller
SafeCartesianPoseController
```

## 当前 controller 核心功能

ROS 框架层已迁移为 ROS2 `controller_interface::ControllerInterface`。

保留的 SERL/Franka controller 核心结构：

- 订阅 `geometry_msgs/msg/PoseStamped` target；
- 非实时 callback 只缓存 target；
- update loop 读取 `q`、`dq`、`O_T_EE`、zero Jacobian、Coriolis；
- 对 raw target 做一阶平滑；
- 以 measured pose 为中心做 position/orientation reference limiting；
- 计算 Cartesian impedance wrench；
- `tau = J^T wrench + tau_nullspace + coriolis`；
- 使用 Franka `tau_J_d` 做 torque rate saturation；
- 输出 7 路 joint effort command。

## 运行指令

构建 overlay：

```bash
cd /home/admin123/WenshuoZhou/SERL
Code/spacemouse_franka_teleop_test/scripts/build_overlay.sh
source Code/spacemouse_franka_teleop_test/scripts/source_ros2_franka_env.sh
```

注意：controller 必须用系统 GCC/libstdc++ 构建。曾经的失败根因为 `CMakeCache.txt` 记录了 conda 编译器：

```text
/home/admin123/miniforge3/bin/x86_64-conda-linux-gnu-c++
```

这会让 plugin 依赖 `CXXABI_1.3.15`，而运行中的 `controller_manager` 使用 Ubuntu 22.04 系统 libstdc++，只提供到 `CXXABI_1.3.13`，最终 `dlopen` 失败。当前 `build_overlay.sh` 已固定：

```bash
CC=/usr/bin/gcc
CXX=/usr/bin/g++
```

并使用 `--cmake-clean-cache`，避免旧 cache 继续污染构建。

检查 plugin type：

```bash
ros2 control list_controller_types | grep -i serl
```

如果 `controller_manager` 是在构建/source 新 overlay 之前启动的，这条命令可能仍然看不到新 type。此时不要继续 spawner；先在 UI 中按 `Clear` 停止 ROS/teleop 进程，再按 `Controller`，让 `franka_bringup` 在新的 overlay 环境中重新启动。

启动 controller：

```bash
Code/spacemouse_franka_teleop_test/scripts/start_serl_cartesian_controller.sh
```

检查 loaded controller：

```bash
ros2 control list_controllers
```

期望看到：

```text
joint_state_broadcaster active
franka_robot_state_broadcaster active
serl_cartesian_impedance_controller active
```

检查 target topic 连接：

```bash
ros2 topic info /serl_cartesian_impedance_controller/target_pose -v
```

应能看到 teleop/test publisher 和 `serl_cartesian_impedance_controller` subscriber。
