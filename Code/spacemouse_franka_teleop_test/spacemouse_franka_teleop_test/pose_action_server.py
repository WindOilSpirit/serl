#!/usr/bin/env python3
"""SpaceMouse action -> SERL-style Cartesian impedance target pose bridge."""

from __future__ import annotations

import csv
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

from .motion_shaping import (
    AxisMotionLimits,
    MotionShaper,
    MotionShapingConfig,
    StepDeltaLimits,
)


def get_array(node: Node, name: str) -> np.ndarray:
    return np.asarray(node.get_parameter(name).value, dtype=np.float64)


def pose_to_array(msg: PoseStamped) -> np.ndarray:
    pose = msg.pose
    return np.array(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        dtype=np.float64,
    )


def array_to_pose_msg(pose: np.ndarray, frame_id: str, node: Node) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(pose[0])
    msg.pose.position.y = float(pose[1])
    msg.pose.position.z = float(pose[2])
    quat = np.asarray(pose[3:7], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if not math.isfinite(norm) or norm < 1e-9:
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    else:
        quat = quat / norm
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


def wrench_norm(msg: WrenchStamped | None) -> tuple[float, float]:
    if msg is None:
        return 0.0, 0.0
    f = msg.wrench.force
    t = msg.wrench.torque
    force = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
    torque = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
    return force, torque


class ImpedanceTeleopServer(Node):
    def __init__(self) -> None:
        super().__init__("spacemouse_franka_impedance_teleop_server")

        self.declare_parameter("action_topic", "/spacemouse_franka_teleop/action")
        self.declare_parameter("deadman_topic", "/spacemouse_franka_teleop/deadman")
        self.declare_parameter("fine_mode_topic", "/spacemouse_franka_teleop/fine_mode")
        self.declare_parameter("target_pose_topic", "/serl_cartesian_impedance_controller/target_pose")
        self.declare_parameter("measured_pose_topic", "/franka_robot_state_broadcaster/current_pose")
        self.declare_parameter("server_status_topic", "/spacemouse_franka_teleop/server_status")
        self.declare_parameter("controller_manager_service", "/controller_manager/list_controllers")
        self.declare_parameter("controller_name", "serl_cartesian_impedance_controller")
        self.declare_parameter(
            "external_wrench_topic",
            "/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame",
        )
        self.declare_parameter("require_active_controller", True)
        self.declare_parameter("publish_enabled", True)
        self.declare_parameter("debug_simulation_mode", False)
        self.declare_parameter("fake_measured_pose_when_debug", False)
        self.declare_parameter("frame_id", "base")
        self.declare_parameter("rate_hz", 250.0)
        self.declare_parameter("log_rate_hz", 2.0)
        self.declare_parameter("debug_csv_path", "/tmp/spacemouse_franka_impedance_trace.csv")

        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("spacemouse_target_scale", 1.0)
        self.declare_parameter("normal_spacemouse_target_scale", 2.0)
        self.declare_parameter("fine_spacemouse_target_scale", 1.0)
        self.declare_parameter("translation_deadzone", 0.06)
        self.declare_parameter("rotation_deadzone", 0.10)
        self.declare_parameter("input_power", 3.0)
        self.declare_parameter("max_action_norm", 1.0)
        self.declare_parameter("command_timeout_s", 0.20)
        self.declare_parameter("measured_pose_timeout_s", 0.20)

        self.declare_parameter("coarse_v_xy_max", 0.003)
        self.declare_parameter("coarse_v_z_up_max", 0.003)
        self.declare_parameter("coarse_v_z_down_max", 0.001)
        self.declare_parameter("coarse_a_xy_max", 0.010)
        self.declare_parameter("coarse_a_z_up_max", 0.010)
        self.declare_parameter("coarse_a_z_down_max", 0.003)
        self.declare_parameter("coarse_j_xy_max", 0.100)
        self.declare_parameter("coarse_j_z_up_max", 0.100)
        self.declare_parameter("coarse_j_z_down_max", 0.030)

        self.declare_parameter("fine_v_xy_max", 0.001)
        self.declare_parameter("fine_v_z_up_max", 0.001)
        self.declare_parameter("fine_v_z_down_max", 0.0005)
        self.declare_parameter("fine_a_xy_max", 0.004)
        self.declare_parameter("fine_a_z_up_max", 0.004)
        self.declare_parameter("fine_a_z_down_max", 0.0015)
        self.declare_parameter("fine_j_xy_max", 0.040)
        self.declare_parameter("fine_j_z_up_max", 0.040)
        self.declare_parameter("fine_j_z_down_max", 0.015)

        self.declare_parameter("d_move", 3.0)
        self.declare_parameter("d_stop", 5.0)
        self.declare_parameter("dt_nominal", 0.004)
        self.declare_parameter("dt_max", 0.020)
        self.declare_parameter("delta_xy_max", 0.000020)
        self.declare_parameter("delta_z_up_max", 0.000020)
        self.declare_parameter("delta_z_down_max", 0.000010)
        self.declare_parameter("workspace_slowdown_distance", 0.030)
        self.declare_parameter("workspace_low", [0.25, -0.20, 0.04])
        self.declare_parameter("workspace_high", [0.75, 0.25, 0.75])

        self.action_topic = self.get_parameter("action_topic").value
        self.deadman_topic = self.get_parameter("deadman_topic").value
        self.fine_mode_topic = self.get_parameter("fine_mode_topic").value
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.measured_pose_topic = self.get_parameter("measured_pose_topic").value
        self.server_status_topic = self.get_parameter("server_status_topic").value
        self.controller_manager_service = self.get_parameter("controller_manager_service").value
        self.controller_name = self.get_parameter("controller_name").value
        self.external_wrench_topic = self.get_parameter("external_wrench_topic").value
        self.require_active_controller = bool(self.get_parameter("require_active_controller").value)
        self.publish_enabled = bool(self.get_parameter("publish_enabled").value)
        self.debug_simulation_mode = bool(self.get_parameter("debug_simulation_mode").value)
        self.fake_measured_pose_when_debug = bool(
            self.get_parameter("fake_measured_pose_when_debug").value
        )
        if self.debug_simulation_mode:
            self.require_active_controller = False
        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.log_rate_hz = float(self.get_parameter("log_rate_hz").value)
        self.debug_csv_path = str(self.get_parameter("debug_csv_path").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.measured_pose_timeout_s = float(self.get_parameter("measured_pose_timeout_s").value)
        self.workspace_low = get_array(self, "workspace_low")
        self.workspace_high = get_array(self, "workspace_high")
        self.motion_shaper = self._make_motion_shaper()

        self.action = np.zeros(3, dtype=np.float64)
        self.rotation_action = np.zeros(3, dtype=np.float64)
        self.deadman = False
        self.fine_mode = False
        self.measured_pose: np.ndarray | None = None
        self.target_pose: np.ndarray | None = None
        self.last_action_time = 0.0
        self.last_measured_time = 0.0
        self.last_tick_time = time.monotonic()
        self.last_log_time = 0.0
        self.target_publish_count = 0
        self.controller_active = self.debug_simulation_mode or not self.require_active_controller
        self.controller_state = "unchecked"
        self.last_controller_check_time = 0.0
        self.pending_controller_future = None
        self.external_wrench: WrenchStamped | None = None
        self.debug_file = None
        self.debug_writer = None

        if self.debug_simulation_mode and self.fake_measured_pose_when_debug:
            self.measured_pose = np.array([0.45, 0.0, 0.20, 0.0, 0.0, 0.0, 1.0])
            self.last_measured_time = time.monotonic()

        self.target_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.server_status_topic, 10)
        self.create_subscription(TwistStamped, self.action_topic, self._action_cb, 10)
        self.create_subscription(Bool, self.deadman_topic, self._deadman_cb, 10)
        self.create_subscription(Bool, self.fine_mode_topic, self._fine_cb, 10)
        self.create_subscription(
            PoseStamped, self.measured_pose_topic, self._measured_cb, qos_profile_sensor_data
        )
        if self.external_wrench_topic:
            self.create_subscription(
                WrenchStamped, self.external_wrench_topic, self._wrench_cb, qos_profile_sensor_data
            )
        self.controller_client = self.create_client(ListControllers, self.controller_manager_service)
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)
        self._open_debug_csv()

        self.get_logger().info("SERL impedance teleop server started")
        self.get_logger().info(f"  target_pose_topic: {self.target_pose_topic}")
        self.get_logger().info(f"  measured_pose_topic: {self.measured_pose_topic}")
        self.get_logger().info(f"  controller_name: {self.controller_name}")
        self.get_logger().info(f"  publish_enabled: {self.publish_enabled}")

    def destroy_node(self) -> bool:
        if self.debug_file is not None:
            self.debug_file.close()
        return super().destroy_node()

    def _make_motion_shaper(self) -> MotionShaper:
        def limits(prefix: str) -> AxisMotionLimits:
            return AxisMotionLimits(
                v_xy=float(self.get_parameter(f"{prefix}_v_xy_max").value),
                v_z_up=float(self.get_parameter(f"{prefix}_v_z_up_max").value),
                v_z_down=float(self.get_parameter(f"{prefix}_v_z_down_max").value),
                a_xy=float(self.get_parameter(f"{prefix}_a_xy_max").value),
                a_z_up=float(self.get_parameter(f"{prefix}_a_z_up_max").value),
                a_z_down=float(self.get_parameter(f"{prefix}_a_z_down_max").value),
                j_xy=float(self.get_parameter(f"{prefix}_j_xy_max").value),
                j_z_up=float(self.get_parameter(f"{prefix}_j_z_up_max").value),
                j_z_down=float(self.get_parameter(f"{prefix}_j_z_down_max").value),
            )

        return MotionShaper(
            MotionShapingConfig(
                speed_scale=self._effective_speed_scale(False),
                translation_deadzone=float(self.get_parameter("translation_deadzone").value),
                rotation_deadzone=float(self.get_parameter("rotation_deadzone").value),
                input_power=float(self.get_parameter("input_power").value),
                max_action_norm=float(self.get_parameter("max_action_norm").value),
                coarse_limits=limits("coarse"),
                fine_limits=limits("fine"),
                d_move=float(self.get_parameter("d_move").value),
                d_stop=float(self.get_parameter("d_stop").value),
                dt_nominal=float(self.get_parameter("dt_nominal").value),
                dt_max=float(self.get_parameter("dt_max").value),
                delta_limits=StepDeltaLimits(
                    xy=float(self.get_parameter("delta_xy_max").value),
                    z_up=float(self.get_parameter("delta_z_up_max").value),
                    z_down=float(self.get_parameter("delta_z_down_max").value),
                ),
                workspace_slowdown_distance=float(
                    self.get_parameter("workspace_slowdown_distance").value
                ),
            )
        )

    def _effective_speed_scale(self, fine_mode: bool) -> float:
        speed_scale = float(self.get_parameter("speed_scale").value)
        global_scale = float(self.get_parameter("spacemouse_target_scale").value)
        mode_scale_name = (
            "fine_spacemouse_target_scale" if fine_mode else "normal_spacemouse_target_scale"
        )
        mode_scale = float(self.get_parameter(mode_scale_name).value)
        return max(0.0, speed_scale * global_scale * mode_scale)

    def _refresh_runtime_motion_scale(self) -> None:
        self.motion_shaper.config.speed_scale = self._effective_speed_scale(self.fine_mode)

    def _action_cb(self, msg: TwistStamped) -> None:
        self.action = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.rotation_action = np.array(
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        )
        self.last_action_time = time.monotonic()

    def _deadman_cb(self, msg: Bool) -> None:
        self.deadman = bool(msg.data)
        if not self.deadman:
            self.motion_shaper.reset()

    def _fine_cb(self, msg: Bool) -> None:
        self.fine_mode = bool(msg.data)

    def _measured_cb(self, msg: PoseStamped) -> None:
        self.measured_pose = pose_to_array(msg)
        self.last_measured_time = time.monotonic()
        if self.target_pose is None:
            self.target_pose = self.measured_pose.copy()

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self.external_wrench = msg

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(1e-4, min(0.05, now - self.last_tick_time))
        self.last_tick_time = now
        self.publish_enabled = bool(self.get_parameter("publish_enabled").value)
        self._refresh_runtime_motion_scale()
        self._update_controller_status(now)

        measured_fresh = (
            self.measured_pose is not None
            and now - self.last_measured_time <= self.measured_pose_timeout_s
        )
        action_fresh = now - self.last_action_time <= self.command_timeout_s
        controller_ready = (not self.require_active_controller) or self.controller_active
        state = "IDLE"
        reason = "ok"

        if self.target_pose is None and self.measured_pose is not None:
            self.target_pose = self.measured_pose.copy()
        if self.target_pose is None:
            state, reason = "WAITING_FOR_MEASURED", "no measured pose"
            self._publish_status(now, state, reason)
            return
        if not measured_fresh:
            state, reason = "WAITING_FOR_MEASURED", "measured pose stale"
            self.motion_shaper.reset()
            self._publish_status(now, state, reason)
            return
        if not controller_ready:
            state, reason = "WAITING_FOR_CONTROLLER", self.controller_state
            self._publish_status(now, state, reason)
            return

        if self.deadman and action_fresh:
            result = self.motion_shaper.step(
                self.action,
                True,
                self.fine_mode,
                dt,
                self.target_pose[:3],
                self.workspace_low,
                self.workspace_high,
            )
            self.target_pose[:3] += result.delta_position
            if self.measured_pose is not None:
                self.target_pose[3:7] = self.measured_pose[3:7]
            state = "HUMAN_CONTROL"
        else:
            self.motion_shaper.step(
                np.zeros(3),
                False,
                self.fine_mode,
                dt,
                self.target_pose[:3],
                self.workspace_low,
                self.workspace_high,
            )

        if self.publish_enabled:
            self.target_pub.publish(array_to_pose_msg(self.target_pose, self.frame_id, self))
            self.target_publish_count += 1
        self._write_debug(now, state, reason)
        self._publish_status(now, state, reason)
        if now - self.last_log_time >= 1.0 / max(0.1, self.log_rate_hz):
            self.last_log_time = now
            self.get_logger().info(
                f"state={state} deadman={self.deadman} target_pub={self.target_publish_count} "
                f"controller={self.controller_state} reason={reason}"
            )

    def _update_controller_status(self, now: float) -> None:
        if not self.require_active_controller:
            self.controller_active = True
            self.controller_state = "not required"
            return
        if self.pending_controller_future is not None:
            return
        if now - self.last_controller_check_time < 1.0:
            return
        self.last_controller_check_time = now
        if not self.controller_client.service_is_ready():
            self.controller_active = False
            self.controller_state = "controller_manager unavailable"
            return
        self.pending_controller_future = self.controller_client.call_async(ListControllers.Request())
        self.pending_controller_future.add_done_callback(self._controller_status_cb)

    def _controller_status_cb(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.controller_active = False
            self.controller_state = f"list_controllers failed: {exc}"
            self.pending_controller_future = None
            return
        self.controller_active = False
        self.controller_state = "not loaded"
        for controller in result.controller:
            if controller.name == self.controller_name:
                self.controller_state = controller.state
                self.controller_active = controller.state == "active"
                break
        self.pending_controller_future = None

    def _status_fields(self, now: float, state: str, reason: str) -> dict[str, object]:
        force_norm, torque_norm = wrench_norm(self.external_wrench)
        target = self.target_pose if self.target_pose is not None else np.full(7, np.nan)
        measured = self.measured_pose if self.measured_pose is not None else np.full(7, np.nan)
        error = target[:3] - measured[:3]
        if not np.all(np.isfinite(error)):
            error_norm = math.nan
        else:
            error_norm = float(np.linalg.norm(error))
        return {
            "state": state,
            "reason": reason,
            "controller_name": self.controller_name,
            "controller_active": self.controller_active,
            "controller_state": self.controller_state,
            "target_pose_topic": self.target_pose_topic,
            "publish_enabled": self.publish_enabled,
            "speed_scale": float(self.get_parameter("speed_scale").value),
            "spacemouse_target_scale": float(self.get_parameter("spacemouse_target_scale").value),
            "normal_spacemouse_target_scale": float(
                self.get_parameter("normal_spacemouse_target_scale").value
            ),
            "fine_spacemouse_target_scale": float(
                self.get_parameter("fine_spacemouse_target_scale").value
            ),
            "effective_speed_scale": self.motion_shaper.config.speed_scale,
            "deadman": self.deadman,
            "fine_mode": self.fine_mode,
            "measured_pose_fresh": now - self.last_measured_time <= self.measured_pose_timeout_s,
            "target_publish_count": self.target_publish_count,
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "measured_x": measured[0],
            "measured_y": measured[1],
            "measured_z": measured[2],
            "target_measured_error_norm": error_norm,
            "raw_action_x": self.action[0],
            "raw_action_y": self.action[1],
            "raw_action_z": self.action[2],
            "external_force_norm": force_norm,
            "external_torque_norm": torque_norm,
        }

    def _publish_status(self, now: float, state: str, reason: str) -> None:
        fields = self._status_fields(now, state, reason)
        msg = String()
        msg.data = " ".join(f"{key}={value}" for key, value in fields.items())
        self.status_pub.publish(msg)

    def _open_debug_csv(self) -> None:
        if not self.debug_csv_path:
            return
        path = Path(self.debug_csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.debug_file = path.open("w", newline="")
        self.debug_writer = csv.DictWriter(
            self.debug_file,
            fieldnames=[
                "time_monotonic",
                "state",
                "reason",
                "controller_active",
                "controller_state",
                "deadman",
                "fine_mode",
                "publish_enabled",
                "speed_scale",
                "spacemouse_target_scale",
                "normal_spacemouse_target_scale",
                "fine_spacemouse_target_scale",
                "effective_speed_scale",
                "target_publish_count",
                "target_x",
                "target_y",
                "target_z",
                "measured_x",
                "measured_y",
                "measured_z",
                "target_measured_error_norm",
                "raw_action_x",
                "raw_action_y",
                "raw_action_z",
                "external_force_norm",
                "external_torque_norm",
            ],
            extrasaction="ignore",
        )
        self.debug_writer.writeheader()

    def _write_debug(self, now: float, state: str, reason: str) -> None:
        if self.debug_writer is None:
            return
        fields = self._status_fields(now, state, reason)
        fields["time_monotonic"] = now
        self.debug_writer.writerow(fields)


def main() -> None:
    rclpy.init()
    node = ImpedanceTeleopServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
