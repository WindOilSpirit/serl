# SERL Franka Controller 集成审计

本文档按当前 workspace 的新结构重新审计控制链路：

```text
hil-serl-main/
serl_franka_controllers-main/
```

结论先行：`serl_franka_controllers-main` 是 ROS1/catkin `ros_control` controller 包，不能直接放进当前 ROS2 / `franka_ros2` / `ros2_control` 控制链里运行。当前应采用方案 A：保留该目录作为原始参考实现，在 ROS2 controller 包中迁移核心 Cartesian impedance control + realtime reference limiting 控制律。

## 目录与包类型

`serl_franka_controllers-main` 包含：

- `package.xml`
- `CMakeLists.txt`
- `serl_franka_controllers_plugin.xml`
- `src/cartesian_impedance_controller.cpp`
- `src/joint_position_controller.cpp`
- `include/serl_franka_controllers/cartesian_impedance_controller.h`
- `include/serl_franka_controllers/joint_position_controller.h`
- `config/serl_franka_controllers.yaml`
- `cfg/compliance_param.cfg`
- `launch/impedance.launch`
- `launch/joint.launch`
- `msg/ZeroJacobian.msg`

`package.xml` 使用 `<buildtool_depend>catkin</buildtool_depend>`，`CMakeLists.txt` 使用 `find_package(catkin REQUIRED COMPONENTS ...)` 和 `generate_dynamic_reconfigure_options(...)`。因此它是 ROS1 包。

## Controller 列表

`serl_franka_controllers_plugin.xml` 注册了两个 controller：

- `serl_franka_controllers/CartesianImpedanceController`
- `serl_franka_controllers/JointPositionController`

`config/serl_franka_controllers.yaml` 中对应的 controller manager 名称是：

- `cartesian_impedance_controller`
- `joint_position_controller`

## Cartesian Impedance Controller

源码：

- `serl_franka_controllers-main/src/cartesian_impedance_controller.cpp`
- `serl_franka_controllers-main/include/serl_franka_controllers/cartesian_impedance_controller.h`

这是 SERL 原本使用的核心低层 controller。它不是 Cartesian pose trajectory follower，而是 Cartesian impedance torque controller。

它的主要机制：

- 继承 ROS1 `controller_interface::MultiInterfaceController`
- 使用 `franka_hw::FrankaModelInterface`
- 使用 `franka_hw::FrankaStateInterface`
- 使用 `hardware_interface::EffortJointInterface`
- 读取 Franka robot state、Jacobian、Coriolis
- 订阅 `equilibrium_pose`
- 内部维护 `position_d_target_ / orientation_d_target_`
- 通过 `filter_params_` 平滑到 `position_d_ / orientation_d_`
- 对 Cartesian position/orientation error 做 realtime clip
- 计算 Cartesian spring-damper wrench
- 通过 `jacobian.transpose()` 转换为 joint torque
- 添加 nullspace torque 和 Coriolis
- 通过 `saturateTorqueRate()` 做 torque rate limiting
- 输出 7 轴 effort command

ROS1 topic：

```text
/cartesian_impedance_controller/equilibrium_pose
/cartesian_impedance_controller/franka_jacobian
```

动态参数由 `dynamic_reconfigure` 提供，定义在 `cfg/compliance_param.cfg`，包括：

- translational stiffness / damping
- rotational stiffness / damping
- nullspace stiffness
- translational clip 正负方向
- rotational clip 正负方向
- integral gain

当前代码中的 reference limiting 是对误差分量分别 clip：

```cpp
error_.head(3) << position - position_d_;
error_(i) = min(max(error_(i), translational_clip_min_(i)), translational_clip_max_(i));
```

迁移到 ROS2 时建议改成按 norm 限制 reference 本身：

```text
error = smoothed_target_position - measured_position

if norm(error) > max_pos_error:
    clipped_target_position = measured_position + max_pos_error * error / norm(error)
    reference_was_clipped = true
else:
    clipped_target_position = smoothed_target_position
    reference_was_clipped = false
```

姿态也应采用从 measured orientation 朝 smoothed target orientation 旋转 `max_ori_error` 的限制方式。

## Joint Position Controller

源码：

- `serl_franka_controllers-main/src/joint_position_controller.cpp`
- `serl_franka_controllers-main/include/serl_franka_controllers/joint_position_controller.h`

用途是 reset / joint position recovery。它使用 ROS1 `hardware_interface::PositionJointInterface`，读取参数 `/target_joint_positions`，在约 10 秒内把关节从初始位置插值到 reset pose。

它不是接触任务控制器，也不是主 teleop / RL controller。

## Launch 与配置

`launch/impedance.launch`：

- include ROS1 `franka_control/launch/franka_control.launch`
- load `config/serl_franka_controllers.yaml`
- spawn `cartesian_impedance_controller`

`launch/joint.launch`：

- include ROS1 `franka_control/launch/franka_control.launch`
- load 同一 YAML
- spawn `joint_position_controller`

这两个 launch 都是 ROS1 XML launch，不能由 ROS2 `ros2 launch` 直接运行。

## ROS 版本与依赖判断

`serl_franka_controllers-main` 依赖：

- `catkin`
- `roscpp`
- `rospy`
- `dynamic_reconfigure`
- `controller_interface`
- `hardware_interface`
- `franka_hw`
- `franka_control`
- `franka_description`
- `franka_gripper`
- `libfranka`
- `pluginlib`
- `realtime_tools`
- `tf`
- `tf_conversions`
- `eigen_conversions`

这些是 ROS1 `franka_ros` / `ros_control` API，不是 ROS2 `franka_ros2` / `ros2_control` API。

当前工程如果实际使用 ROS2 Humble、`controller_manager`、`ros2_control_node` 和 `franka_ros2`，则不能直接编译或加载该包。

## HIL-SERL 上层接法

`hil-serl-main/serl_robot_infra/robot_servers/franka_server.py` 是 ROS1 `rospy` + Flask robot server。它：

- 启动 `roscore`
- `roslaunch serl_franka_controllers impedance.launch`
- 向 `/cartesian_impedance_controller/equilibrium_pose` 发布 `geometry_msgs/PoseStamped`
- 订阅 `/cartesian_impedance_controller/franka_jacobian`
- 订阅 `franka_state_controller/franka_states`
- 用 `dynamic_reconfigure.client.Client` 修改 compliance / clip 参数
- 通过 Flask HTTP endpoint 暴露 `/pose`, `/getpos`, `/getforce`, `/getvel`, `/update_param` 等接口

也就是说，HIL-SERL 原始路线是：

```text
HIL-SERL env / actor / human intervention
-> Flask robot server
-> ROS1 equilibrium_pose topic + dynamic_reconfigure
-> SERL Cartesian impedance controller
-> franka_ros / libfranka
-> Franka
```

不是：

```text
target_pose -> 1 kHz Cartesian pose follower -> franka cartesian_pose_command
```

## 当前不能直接集成的接口

从 ROS1 迁移到 ROS2 时必须替换：

- `controller_interface::MultiInterfaceController` -> ROS2 `controller_interface::ControllerInterface`
- `hardware_interface::EffortJointInterface` -> ROS2 command interfaces for joint effort
- `franka_hw::FrankaModelInterface` -> `franka_semantic_components` / `franka_ros2` model semantic component
- `franka_hw::FrankaStateInterface` -> ROS2 Franka state semantic component / state interfaces
- `ros::Subscriber` -> `rclcpp` subscription with realtime buffer
- `dynamic_reconfigure` -> ROS2 parameters / parameter callback / services
- ROS1 plugin XML export -> ROS2 `pluginlib_export_plugin_description_file(controller_interface ...)`
- ROS1 XML launch -> ROS2 Python launch
- `franka_msgs/FrankaState` topic assumptions -> ROS2 broadcaster topics/interfaces
- `ZeroJacobian.msg` publication -> ROS2 message or standard matrix/debug topic

## 选择的集成方案

采用方案 A。

理由：

- `serl_franka_controllers-main` 是 ROS1/catkin 包。
- 当前系统使用 ROS2 / `ros2_control` / `franka_ros2`。
- 直接把 ROS1 controller 放入 ROS2 workspace 会在 build system、controller interface、hardware interface、launch、dynamic parameter 机制上全部不兼容。

保留 `serl_franka_controllers-main` 作为原始参考实现，不应直接修改成半 ROS1 半 ROS2 的混合包。

## 推荐 ROS2 迁移目标

建议在 ROS2 controller 包中新增：

```text
serl_franka_ros2_control/src/serl_cartesian_impedance_controller.cpp
serl_franka_ros2_control/include/serl_franka_ros2_control/serl_cartesian_impedance_controller.hpp
serl_franka_ros2_control/config/serl_cartesian_impedance_controller.yaml
serl_franka_ros2_control/launch/serl_cartesian_impedance_controller.launch.py
```

controller 名称建议：

```text
serl_cartesian_impedance_controller
```

类型建议：

```text
serl_franka_ros2_control/SerlCartesianImpedanceController
```

## ROS2 Controller 必须保留的核心机制

新的 ROS2 controller 应实现：

- Cartesian impedance control
- realtime reference limiting
- 高层 target / delta pose 输入
- 内部维护 raw target、smoothed target、clipped target、measured pose
- 每个 1 kHz update 中读取 measured EE pose、Jacobian、Coriolis、robot model
- 对 raw target 做平滑
- 对 smoothed target 做 reference limiting
- 计算 Cartesian wrench
- 转换为 joint torque
- 加 Coriolis / nullspace torque
- torque rate saturation
- 输出 joint torque command

debug 输出至少包括：

- raw target pose
- smoothed target pose
- clipped target pose
- measured pose
- reference 是否被 clip
- clip 前 position/orientation error
- clip 后 position/orientation error
- commanded torque
- estimated external wrench / force，如果可从 Franka state 取得

## 当前工程接入建议

短期：

- 停止默认使用旧 `SafeCartesianPoseController`。
- 停止默认运行旧 Test0 / Test1 pose follower 测试。
- 停止让 SpaceMouse / HIL-SERL 上层默认发布到 `/serl_safe_cartesian_pose_controller/target_pose`。
- 把 `serl_franka_controllers-main` 仅作为 ROS1 参考，不加入 ROS2 build。

中期：

- 新增 ROS2 `SerlCartesianImpedanceController`。
- controller 订阅一个 ROS2 target pose topic，例如：

```text
/serl_cartesian_impedance_controller/target_pose
```

- controller 发布 debug topics，例如：

```text
/serl_cartesian_impedance_controller/debug/raw_target_pose
/serl_cartesian_impedance_controller/debug/smoothed_target_pose
/serl_cartesian_impedance_controller/debug/clipped_target_pose
/serl_cartesian_impedance_controller/debug/measured_pose
/serl_cartesian_impedance_controller/debug/status
```

- 迁移 `hil-serl-main/serl_robot_infra/robot_servers/franka_server.py` 的 ROS1 `rospy` 入口，或新增 ROS2 robot server，使其向新的 impedance target topic 发 pose / delta pose，并通过 ROS2 parameter service 设置 stiffness、damping、clip 限制。

长期：

- 让 HIL-SERL actor / learner / human intervention 只依赖 robot server API，不直接耦合旧 pose follower topic。
- 让 bottom controller 的默认启动路径统一指向 SERL-style Cartesian impedance controller。
