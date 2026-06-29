# DEPRECATED: old Codex-generated pose follower route

本目录只保留历史文件，不能作为后续 SERL/HIL-SERL 默认控制路线。

这里的内容依赖旧的 `SafeCartesianPoseController`：

```text
target_pose -> internal_command -> measured_pose
```

这不是 SERL 原始 Franka controller。SERL/HIL-SERL 后续应迁移到：

```text
Cartesian impedance controller with realtime reference limiting
```

旧 Test1、旧 SpaceMouse pose follower README、旧伪代码和旧参数只用于追溯此前 discontinuity 调试过程。不要再向：

```text
/serl_safe_cartesian_pose_controller/target_pose
```

发布新测试命令。

新的集成审计见：

```text
docs/serl_controller_integration_audit.md
```
