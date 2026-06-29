#!/usr/bin/env python3
"""Controller-only preset Cartesian trajectory test for SafeCartesianPoseController."""

from __future__ import annotations

import csv
import math
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, Optional

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


NAN = float("nan")

STATUS_FLOAT_FIELDS = [
    "translation_tracking_time_s",
    "max_translation_speed_mps",
    "max_translation_acceleration_mps2",
    "max_translation_jerk_mps3",
    "max_translation_step_m",
    "max_target_distance_m",
    "watchdog_timeout_sec",
    "raw_target_to_command_error_norm",
    "raw_target_to_command_error_x",
    "raw_target_to_command_error_y",
    "raw_target_to_command_error_z",
    "desired_position_before_guard_x",
    "desired_position_before_guard_y",
    "desired_position_before_guard_z",
    "desired_position_after_guard_x",
    "desired_position_after_guard_y",
    "desired_position_after_guard_z",
    "desired_velocity_x",
    "desired_velocity_y",
    "desired_velocity_z",
    "desired_velocity_norm",
    "desired_acceleration_x",
    "desired_acceleration_y",
    "desired_acceleration_z",
    "desired_acceleration_norm",
    "acceleration_delta_norm",
    "command_velocity_x",
    "command_velocity_y",
    "command_velocity_z",
    "command_velocity_norm",
    "command_acceleration_x",
    "command_acceleration_y",
    "command_acceleration_z",
    "command_acceleration_norm",
    "step_x",
    "step_y",
    "step_z",
    "step_norm",
    "command_measured_error_x",
    "command_measured_error_y",
    "command_measured_error_z",
    "command_measured_error_norm",
    "target_measured_error_norm",
]

STATUS_BOOL_FIELDS = [
    "target_distance_clamped",
    "desired_speed_limited",
    "desired_acceleration_limited",
    "jerk_limited",
    "step_limited",
]


def quat_norm(q: Iterable[float]) -> float:
    qx, qy, qz, qw = q
    return math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)


def normalize_quat(q: Iterable[float]) -> list[float]:
    values = [float(v) for v in q]
    norm = quat_norm(values)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError("Invalid quaternion")
    return [v / norm for v in values]


def quat_dot(a: Iterable[float], b: Iterable[float]) -> float:
    return sum(float(x) * float(y) for x, y in zip(a, b))


def quat_angle_error(a: Iterable[float], b: Iterable[float]) -> float:
    qa = normalize_quat(a)
    qb = normalize_quat(b)
    dot = min(1.0, max(-1.0, abs(quat_dot(qa, qb))))
    return 2.0 * math.acos(dot)


def pose_to_xyz_quat(msg: Optional[PoseStamped]) -> tuple[list[float], list[float]]:
    if msg is None:
        return [NAN, NAN, NAN], [NAN, NAN, NAN, NAN]
    p = msg.pose.position
    q = msg.pose.orientation
    return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]


def finite_xyz(xyz: Iterable[float]) -> bool:
    return all(math.isfinite(float(v)) for v in xyz)


def norm3(values: Iterable[float]) -> float:
    vals = [float(v) for v in values]
    return math.sqrt(vals[0] * vals[0] + vals[1] * vals[1] + vals[2] * vals[2])


def parse_status(data: str) -> Dict[str, str]:
    return dict(re.findall(r"([A-Za-z0-9_]+)=([^ ]+)", data or ""))


def as_float(mapping: Dict[str, str], key: str) -> float:
    try:
        return float(mapping.get(key, "nan"))
    except ValueError:
        return NAN


def as_int(mapping: Dict[str, str], key: str) -> int:
    try:
        return int(mapping.get(key, "0"))
    except ValueError:
        return 0


@dataclass
class TimedPose:
    msg: Optional[PoseStamped] = None
    stamp_monotonic: float = 0.0
    update_count: int = 0

    def age(self) -> float:
        if self.msg is None:
            return NAN
        return time.monotonic() - self.stamp_monotonic

    def fresh(self, max_age_s: float) -> bool:
        return self.msg is not None and self.age() <= max_age_s


class PresetTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__("test1_preset_cartesian_trajectory_node")
        self._declare_parameters()
        self._load_parameters()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.measured = TimedPose()
        self.internal = TimedPose()
        self.accepted = TimedPose()
        self.rt_target = TimedPose()
        self.status: Dict[str, str] = {}
        self.status_age_stamp = 0.0
        self.status_update_count = 0

        self.target_pub = self.create_publisher(PoseStamped, self.target_topic, target_qos)
        self.create_subscription(PoseStamped, self.measured_pose_topic, self._measured_cb, qos)
        self.create_subscription(PoseStamped, self.internal_command_topic, self._internal_cb, qos)
        self.create_subscription(PoseStamped, self.accepted_target_topic, self._accepted_cb, qos)
        self.create_subscription(PoseStamped, self.rt_target_topic, self._rt_target_cb, qos)
        self.create_subscription(String, self.controller_status_topic, self._status_cb, qos)

        self.controller_client = self.create_client(ListControllers, self.controller_manager_service)
        self.enable_targets_client = self.create_client(Trigger, self.enable_targets_service)

        self.prev_target_x = NAN
        self.prev_target_vx = NAN
        self.prev_target_ax = NAN
        self.prev_internal_x = NAN
        self.prev_internal_vx = NAN
        self.prev_internal_ax = NAN
        self.prev_target_q: Optional[list[float]] = None
        self.publish_count = 0
        self.rows: list[dict[str, object]] = []
        self._controller_cache_time = 0.0
        self._controller_cache = (False, False, "not checked")

    def fail(self, code: int, message: str, hints: Optional[list[str]] = None) -> int:
        """Print a clear preflight failure reason before exiting with a stable code."""
        hints = hints or []
        self.get_logger().error(message)
        print("", file=sys.stderr)
        print(f"[Test1 预检查失败] exit code {code}", file=sys.stderr)
        print(message, file=sys.stderr)
        for hint in hints:
            print(f"- {hint}", file=sys.stderr)
        print("", file=sys.stderr)
        return code

    def _declare_parameters(self) -> None:
        defaults = {
            "target_topic": "/serl_safe_cartesian_pose_controller/target_pose",
            "measured_pose_topic": "/franka_robot_state_broadcaster/current_pose",
            "internal_command_topic": "/serl_safe_cartesian_pose_controller/debug/internal_command_pose",
            "accepted_target_topic": "/serl_safe_cartesian_pose_controller/debug/accepted_target_pose",
            "rt_target_topic": "/serl_safe_cartesian_pose_controller/debug/rt_target_pose",
            "controller_status_topic": "/serl_safe_cartesian_pose_controller/debug/target_status",
            "controller_manager_service": "/controller_manager/list_controllers",
            "enable_targets_service": "/serl_safe_cartesian_pose_controller/enable_targets",
            "controller_name": "serl_safe_cartesian_pose_controller",
            "frame_id": "fr3_link0",
            "publish_rate_hz": 1000.0,
            "axis": "x",
            "amplitude_m": 0.0005,
            "frequency_hz": 0.10,
            "third_harmonic_ratio": 0.3,
            "smooth_envelope_s": 2.0,
            "hold_before_s": 2.0,
            "motion_duration_s": 20.0,
            "hold_after_s": 2.0,
            "max_v_ref_allowed": 0.05,
            "max_a_ref_allowed": 0.2,
            "max_j_ref_allowed": 1,
            "max_pose_age_s": 0.5,
            "max_initial_command_measured_m": 0.00050,
            "initial_settle_s": 2.0,
            "output_dir": "",
            "output_csv": "",
            "call_enable_targets": True,
            "require_single_publisher": True,
            "require_controller_subscriber": True,
            "controller_wait_timeout_s": 5.0,
            "state_wait_timeout_s": 10.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)

    def _load_parameters(self) -> None:
        gp = self.get_parameter
        self.target_topic = gp("target_topic").value
        self.measured_pose_topic = gp("measured_pose_topic").value
        self.internal_command_topic = gp("internal_command_topic").value
        self.accepted_target_topic = gp("accepted_target_topic").value
        self.rt_target_topic = gp("rt_target_topic").value
        self.controller_status_topic = gp("controller_status_topic").value
        self.controller_manager_service = gp("controller_manager_service").value
        self.enable_targets_service = gp("enable_targets_service").value
        self.controller_name = gp("controller_name").value
        self.frame_id = gp("frame_id").value
        self.publish_rate_hz = float(gp("publish_rate_hz").value)
        self.axis = str(gp("axis").value).lower()
        self.amplitude_m = float(gp("amplitude_m").value)
        self.frequency_hz = float(gp("frequency_hz").value)
        self.third_harmonic_ratio = float(gp("third_harmonic_ratio").value)
        self.smooth_envelope_s = float(gp("smooth_envelope_s").value)
        self.hold_before_s = float(gp("hold_before_s").value)
        self.motion_duration_s = float(gp("motion_duration_s").value)
        self.hold_after_s = float(gp("hold_after_s").value)
        self.max_v_ref_allowed = float(gp("max_v_ref_allowed").value)
        self.max_a_ref_allowed = float(gp("max_a_ref_allowed").value)
        self.max_j_ref_allowed = float(gp("max_j_ref_allowed").value)
        self.max_pose_age_s = float(gp("max_pose_age_s").value)
        self.max_initial_command_measured_m = float(gp("max_initial_command_measured_m").value)
        self.initial_settle_s = float(gp("initial_settle_s").value)
        output_dir_value = str(gp("output_dir").value)
        if output_dir_value:
            self.output_dir = Path(output_dir_value)
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_dir = Path("debug_output") / f"test1_{stamp}"
        self.output_csv_param = str(gp("output_csv").value)
        self.call_enable_targets = bool(gp("call_enable_targets").value)
        self.require_single_publisher = bool(gp("require_single_publisher").value)
        self.require_controller_subscriber = bool(gp("require_controller_subscriber").value)
        self.controller_wait_timeout_s = float(gp("controller_wait_timeout_s").value)
        self.state_wait_timeout_s = float(gp("state_wait_timeout_s").value)

        if self.axis not in ("x", "y", "z"):
            raise ValueError("axis must be x, y, or z")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

    def _measured_cb(self, msg: PoseStamped) -> None:
        self.measured = TimedPose(msg, time.monotonic(), self.measured.update_count + 1)

    def _internal_cb(self, msg: PoseStamped) -> None:
        self.internal = TimedPose(msg, time.monotonic(), self.internal.update_count + 1)

    def _accepted_cb(self, msg: PoseStamped) -> None:
        self.accepted = TimedPose(msg, time.monotonic(), self.accepted.update_count + 1)

    def _rt_target_cb(self, msg: PoseStamped) -> None:
        self.rt_target = TimedPose(msg, time.monotonic(), self.rt_target.update_count + 1)

    def _status_cb(self, msg: String) -> None:
        self.status = parse_status(msg.data)
        self.status_age_stamp = time.monotonic()
        self.status_update_count += 1

    def spin_until(self, predicate, timeout_s: float, description: str) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        self.get_logger().error(f"Timed out waiting for {description}")
        return False

    def reference_limits(self) -> tuple[float, float, float]:
        w = 2.0 * math.pi * self.frequency_hz
        samples = 20000
        max_v = 0.0
        max_a = 0.0
        max_j = 0.0
        for i in range(samples + 1):
            t = self.motion_duration_s * i / samples
            _, v, a, j = self.reference_scalar(t)
            max_v = max(max_v, abs(v))
            max_a = max(max_a, abs(a))
            max_j = max(max_j, abs(j))
        analytic_v_bound = abs(self.amplitude_m * w) + abs(self.third_harmonic_ratio * self.amplitude_m * 3.0 * w)
        analytic_j_bound = abs(self.amplitude_m * w**3) + abs(self.third_harmonic_ratio * self.amplitude_m * (3.0 * w) ** 3)
        return max(max_v, analytic_v_bound), max_a, max(max_j, analytic_j_bound)

    def reference_scalar(self, t: float) -> tuple[float, float, float, float]:
        w = 2.0 * math.pi * self.frequency_hz
        a = self.amplitude_m
        r = self.third_harmonic_ratio
        base = a * math.sin(w * t) + r * a * math.sin(3.0 * w * t)
        base_v = a * w * math.cos(w * t) + r * a * 3.0 * w * math.cos(3.0 * w * t)
        base_a = -a * w * w * math.sin(w * t) - r * a * (3.0 * w) ** 2 * math.sin(3.0 * w * t)
        base_j = -a * w**3 * math.cos(w * t) - r * a * (3.0 * w) ** 3 * math.cos(3.0 * w * t)
        env, env_v, env_a, env_j = self.smooth_envelope(t)
        x = env * base
        v = env_v * base + env * base_v
        acc = env_a * base + 2.0 * env_v * base_v + env * base_a
        jerk = env_j * base + 3.0 * env_a * base_v + 3.0 * env_v * base_a + env * base_j
        return x, v, acc, jerk

    def smooth_envelope(self, t: float) -> tuple[float, float, float, float]:
        ramp = max(0.0, min(self.smooth_envelope_s, 0.5 * self.motion_duration_s))
        if ramp <= 0.0:
            return 1.0, 0.0, 0.0, 0.0
        if t < ramp:
            return self._smoothstep_with_derivatives(t / ramp, ramp, sign=1.0)
        if t > self.motion_duration_s - ramp:
            return self._smoothstep_with_derivatives((self.motion_duration_s - t) / ramp, ramp, sign=-1.0)
        return 1.0, 0.0, 0.0, 0.0

    def _smoothstep_with_derivatives(self, u: float, ramp: float, sign: float) -> tuple[float, float, float, float]:
        u = min(1.0, max(0.0, u))
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u
        value = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
        du = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
        d2u = 60.0 * u - 180.0 * u2 + 120.0 * u3
        d3u = 60.0 - 360.0 * u + 360.0 * u2
        first = sign * du / ramp
        second = d2u / (ramp * ramp)
        third = sign * d3u / (ramp * ramp * ramp)
        return value, first, second, third

    def controller_active(self) -> tuple[bool, bool, str]:
        now = time.monotonic()
        if now - self._controller_cache_time < 0.5:
            return self._controller_cache
        if not self.controller_client.wait_for_service(timeout_sec=self.controller_wait_timeout_s):
            self._controller_cache = (False, False, "controller_manager unavailable")
            self._controller_cache_time = now
            return self._controller_cache
        future = self.controller_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.controller_wait_timeout_s)
        if not future.done() or future.result() is None:
            self._controller_cache = (True, False, "list_controllers timeout")
            self._controller_cache_time = now
            return self._controller_cache
        for controller in future.result().controller:
            if controller.name == self.controller_name:
                self._controller_cache = (True, controller.state == "active", controller.state)
                self._controller_cache_time = now
                return self._controller_cache
        self._controller_cache = (True, False, "not loaded")
        self._controller_cache_time = now
        return self._controller_cache

    def call_enable_service(self) -> bool:
        if not self.call_enable_targets:
            return True
        if not self.enable_targets_client.wait_for_service(timeout_sec=self.controller_wait_timeout_s):
            self.get_logger().error(f"Enable-targets service unavailable: {self.enable_targets_service}")
            return False
        future = self.enable_targets_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.controller_wait_timeout_s)
        if not future.done() or future.result() is None:
            self.get_logger().error("Enable-targets service timed out")
            return False
        result = future.result()
        if not result.success:
            self.get_logger().error(f"Enable-targets service rejected test: {result.message}")
            return False
        self.get_logger().info(f"Enable-targets service succeeded: {result.message}")
        return True

    def make_pose(self, xyz: list[float], quat: list[float]) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def target_reference(self, base_xyz: list[float], t_motion: float) -> tuple[list[float], list[float], list[float], list[float]]:
        offset, vel, acc, jerk = self.reference_scalar(t_motion)
        xyz = list(base_xyz)
        v = [0.0, 0.0, 0.0]
        a = [0.0, 0.0, 0.0]
        j = [0.0, 0.0, 0.0]
        axis_index = {"x": 0, "y": 1, "z": 2}[self.axis]
        xyz[axis_index] += offset
        v[axis_index] = vel
        a[axis_index] = acc
        j[axis_index] = jerk
        return xyz, v, a, j

    def write_csv(self) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = (
            Path(self.output_csv_param)
            if self.output_csv_param
            else self.output_dir / "test1_preset_cartesian_trajectory.csv"
        )
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.rows:
            return csv_path
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        return csv_path

    def generate_plots(self, csv_path: Path) -> None:
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover - depends on local ROS env
            self.get_logger().warn(f"Skipping plots because matplotlib is unavailable: {exc}")
            return

        def series(name: str) -> list[float]:
            return [float(row.get(name, NAN)) for row in self.rows]

        t = series("t")

        def save(name: str, plots: list[tuple[str, str]], hlines: Optional[list[float]] = None) -> None:
            plt.figure(figsize=(11, 6))
            for field, label in plots:
                plt.plot(t, series(field), label=label)
            if hlines:
                for value in hlines:
                    plt.axhline(value, color="k", linestyle="--", linewidth=0.8, alpha=0.45)
            plt.grid(True, alpha=0.3)
            plt.xlabel("time [s]")
            plt.legend()
            plt.tight_layout()
            plt.savefig(self.output_dir / name)
            plt.close()

        save(
            "positions.png",
            [
                ("x_ref", "x_ref"),
                ("target_x", "target_x"),
                ("controller_internal_command_x", "controller_internal_command_x"),
                ("measured_x", "measured_x"),
            ],
        )
        save(
            "tracking_errors.png",
            [
                ("target_minus_internal_norm", "target_minus_internal_norm"),
                ("internal_minus_measured_norm", "internal_minus_measured_norm"),
                ("target_minus_measured_norm", "target_minus_measured_norm"),
            ],
            hlines=[0.0001, 0.0003, 0.0005],
        )
        save("reference_vaj.png", [("vx_ref", "vx_ref"), ("ax_ref", "ax_ref"), ("jx_ref", "jx_ref")])
        save(
            "target_backderived_vaj.png",
            [
                ("target_v_backderived_x", "target_v_backderived_x"),
                ("target_a_backderived_x", "target_a_backderived_x"),
                ("target_j_backderived_x", "target_j_backderived_x"),
            ],
        )
        save(
            "controller_internal_vaj.png",
            [
                ("controller_internal_v_backderived_x", "controller_internal_v_backderived_x"),
                ("controller_internal_a_backderived_x", "controller_internal_a_backderived_x"),
                ("controller_internal_j_backderived_x", "controller_internal_j_backderived_x"),
            ],
        )
        save(
            "orientation_continuity.png",
            [
                ("target_quat_dot_prev", "target_quat_dot_prev"),
                ("orientation_error_angle_rad", "orientation_error_angle_rad"),
            ],
        )
        save(
            "publish_timing.png",
            [
                ("dt", "dt"),
                ("publish_rate_hz_observed", "publish_rate_hz_observed"),
                ("controller_update_period_s", "controller_update_period_s"),
            ],
        )
        self.get_logger().info(f"Wrote plots next to CSV: {csv_path.parent}")

    def run_test(self) -> int:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        max_v, max_a, max_j = self.reference_limits()
        self.get_logger().info(
            "Reference limits: max|v|=%.9f m/s, max|a|=%.9f m/s^2, max|j|=%.9f m/s^3"
            % (max_v, max_a, max_j)
        )
        if max_v > self.max_v_ref_allowed or max_a > self.max_a_ref_allowed or max_j > self.max_j_ref_allowed:
            return self.fail(
                2,
                "参考轨迹的速度/加速度/jerk 超过当前安全上限，拒绝开始。",
                [
                    "降低 amplitude_m 或 frequency_hz。",
                    "确认 max_v_ref_allowed、max_a_ref_allowed、max_j_ref_allowed 是否符合本次测试计划。",
                ],
            )

        if not self.spin_until(lambda: self.measured.fresh(self.max_pose_age_s), self.state_wait_timeout_s, "fresh measured pose"):
            return self.fail(
                3,
                "等待 measured pose 超时。",
                [
                    "确认 /franka_robot_state_broadcaster/current_pose 正在发布。",
                    "确认 Franka bringup 仍在运行。",
                ],
            )
        if not self.spin_until(lambda: self.internal.fresh(self.max_pose_age_s), self.state_wait_timeout_s, "fresh controller internal command pose"):
            return self.fail(
                4,
                "等待 controller internal command pose 超时。",
                [
                    "确认 /serl_safe_cartesian_pose_controller/debug/internal_command_pose 正在发布。",
                    "确认 serl_safe_cartesian_pose_controller 已 active。",
                    "确认 controller 参数 debug_publish_period_s 大于 0。",
                ],
            )

        controller_manager_available, active, state = self.controller_active()
        if not controller_manager_available or not active:
            return self.fail(
                5,
                f"controller 未处于 active 状态：{state}",
                [
                    "运行 Code/spacemouse_franka_teleop_test/scripts/list_controllers.sh 检查状态。",
                    "确认终端 2 已启动 start_serl_cartesian_controller.sh。",
                ],
            )

        if self.initial_settle_s > 0.0:
            self.get_logger().info(
                f"Controller is active. Waiting {self.initial_settle_s:.3f}s for measured/internal poses to settle."
            )
            settle_end = time.monotonic() + self.initial_settle_s
            while rclpy.ok() and time.monotonic() < settle_end:
                rclpy.spin_once(self, timeout_sec=0.02)

        internal_xyz, internal_q = pose_to_xyz_quat(self.internal.msg)
        measured_xyz, measured_q = pose_to_xyz_quat(self.measured.msg)
        initial_error = norm3([internal_xyz[i] - measured_xyz[i] for i in range(3)])
        self.get_logger().info(f"Initial internal-command minus measured norm: {initial_error:.9f} m")
        if initial_error > self.max_initial_command_measured_m:
            return self.fail(
                6,
                (
                    "启动前 controller_internal_command_pose 与 measured_pose 距离过大："
                    f"{initial_error:.9f} m，当前上限 {self.max_initial_command_measured_m:.9f} m。"
                ),
                [
                    "这是安全预检查失败，所以 Test1 没有发布运动轨迹。",
                    "先等待 3 到 5 秒，让 controller hold 稳定后重新运行 Test1。",
                    "确认没有 pose_action_server、SpaceMouse teleop 或其他节点在发布 target。",
                    "当前默认值与 controller 的 command_measured_tracking_tolerance_m 对齐；如果仍失败，controller 自己也大概率会拒绝 first target。",
                ],
            )

        base_xyz = list(internal_xyz)
        target_q = normalize_quat(measured_q)
        if quat_dot(target_q, internal_q) < 0.0:
            target_q = [-v for v in target_q]

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            pub_count = self.count_publishers(self.target_topic)
            sub_count = self.count_subscribers(self.target_topic)
            if (not self.require_single_publisher or pub_count == 1) and (
                not self.require_controller_subscriber or sub_count >= 1
            ):
                break
        pub_count = self.count_publishers(self.target_topic)
        sub_count = self.count_subscribers(self.target_topic)
        if self.require_single_publisher and pub_count != 1:
            return self.fail(
                7,
                f"{self.target_topic} publisher 数量为 {pub_count}，期望只有 Test1 这一个 publisher。",
                [
                    "关闭 pose_action_server、spacemouse teleop 或其他 target publisher。",
                    f"运行 ros2 topic info {self.target_topic} 查看是谁在发布。",
                ],
            )
        if self.require_controller_subscriber and sub_count < 1:
            return self.fail(
                8,
                f"{self.target_topic} 没有 subscriber，controller 可能未启动或 topic 名称不匹配。",
                [
                    "确认 serl_safe_cartesian_pose_controller 已 active。",
                    "确认 target_topic 参数是否与 controller 实际 target topic 一致。",
                ],
            )

        if not self.call_enable_service():
            return self.fail(
                9,
                "调用 controller enable_targets 服务失败。",
                [
                    f"确认服务存在：ros2 service list | grep {self.enable_targets_service}",
                    "检查 controller 日志中是否拒绝 enable_targets。",
                ],
            )

        period = 1.0 / self.publish_rate_hz
        total_duration = self.hold_before_s + self.motion_duration_s + self.hold_after_s
        start = time.monotonic()
        next_tick = start
        last_tick = start
        phase = "HOLD_BEFORE"
        self.prev_target_q = list(target_q)
        self.get_logger().info(
            f"Publishing preset trajectory for {total_duration:.3f}s at {self.publish_rate_hz:.1f} Hz"
        )

        while rclpy.ok():
            now = time.monotonic()
            if now < next_tick:
                rclpy.spin_once(self, timeout_sec=min(0.001, max(0.0, next_tick - now)))
                continue
            rclpy.spin_once(self, timeout_sec=0.0)
            elapsed = now - start
            dt = now - last_tick if self.publish_count > 0 else period
            last_tick = now

            if elapsed < self.hold_before_s:
                phase = "HOLD_BEFORE"
                target_xyz = list(base_xyz)
                ref_v = [0.0, 0.0, 0.0]
                ref_a = [0.0, 0.0, 0.0]
                ref_j = [0.0, 0.0, 0.0]
            elif elapsed < self.hold_before_s + self.motion_duration_s:
                phase = "MOTION"
                t_motion = elapsed - self.hold_before_s
                target_xyz, ref_v, ref_a, ref_j = self.target_reference(base_xyz, t_motion)
            elif elapsed < total_duration:
                phase = "HOLD_AFTER"
                target_xyz, _, _, _ = self.target_reference(base_xyz, self.motion_duration_s)
                ref_v = [0.0, 0.0, 0.0]
                ref_a = [0.0, 0.0, 0.0]
                ref_j = [0.0, 0.0, 0.0]
            else:
                phase = "DONE"
                break

            q_dot_prev = quat_dot(target_q, self.prev_target_q or target_q)
            if q_dot_prev < 0.0:
                target_q = [-v for v in target_q]
                q_dot_prev = quat_dot(target_q, self.prev_target_q or target_q)
            self.prev_target_q = list(target_q)

            msg = self.make_pose(target_xyz, target_q)
            self.target_pub.publish(msg)
            self.publish_count += 1
            row = self.build_row(elapsed, dt, phase, target_xyz, target_q, ref_v, ref_a, ref_j, q_dot_prev)
            self.rows.append(row)
            next_tick += period
            if next_tick < time.monotonic() - period:
                next_tick = time.monotonic()

        csv_path = self.write_csv()
        self.generate_plots(csv_path)
        self.get_logger().info(f"Test complete. CSV: {csv_path}")
        return 0

    def build_row(
        self,
        elapsed: float,
        dt: float,
        phase: str,
        target_xyz: list[float],
        target_q: list[float],
        ref_v: list[float],
        ref_a: list[float],
        ref_j: list[float],
        target_quat_dot_prev: float,
    ) -> dict[str, object]:
        measured_xyz, measured_q = pose_to_xyz_quat(self.measured.msg)
        internal_xyz, internal_q = pose_to_xyz_quat(self.internal.msg)
        accepted_xyz, _ = pose_to_xyz_quat(self.accepted.msg)
        rt_xyz, _ = pose_to_xyz_quat(self.rt_target.msg)

        target_delta = [target_xyz[i] - internal_xyz[i] for i in range(3)] if finite_xyz(internal_xyz) else [NAN, NAN, NAN]
        tmi = [target_xyz[i] - internal_xyz[i] for i in range(3)] if finite_xyz(internal_xyz) else [NAN, NAN, NAN]
        imm = [internal_xyz[i] - measured_xyz[i] for i in range(3)] if finite_xyz(internal_xyz) and finite_xyz(measured_xyz) else [NAN, NAN, NAN]
        tmm = [target_xyz[i] - measured_xyz[i] for i in range(3)] if finite_xyz(measured_xyz) else [NAN, NAN, NAN]

        target_vx = (target_xyz[0] - self.prev_target_x) / dt if math.isfinite(self.prev_target_x) and dt > 0 else NAN
        target_ax = (target_vx - self.prev_target_vx) / dt if math.isfinite(self.prev_target_vx) and dt > 0 else NAN
        target_jx = (target_ax - self.prev_target_ax) / dt if math.isfinite(self.prev_target_ax) and dt > 0 else NAN
        self.prev_target_x = target_xyz[0]
        self.prev_target_vx = target_vx
        self.prev_target_ax = target_ax

        internal_vx = (internal_xyz[0] - self.prev_internal_x) / dt if math.isfinite(self.prev_internal_x) and dt > 0 else NAN
        internal_ax = (internal_vx - self.prev_internal_vx) / dt if math.isfinite(self.prev_internal_vx) and dt > 0 else NAN
        internal_jx = (internal_ax - self.prev_internal_ax) / dt if math.isfinite(self.prev_internal_ax) and dt > 0 else NAN
        self.prev_internal_x = internal_xyz[0]
        self.prev_internal_vx = internal_vx
        self.prev_internal_ax = internal_ax

        status_age = time.monotonic() - self.status_age_stamp if self.status_update_count else NAN
        controller_manager_available, controller_active, controller_state = self._controller_cache
        error_reason = ""
        franka_error_detected = False
        if not self.measured.fresh(self.max_pose_age_s):
            franka_error_detected = True
            error_reason = "measured_pose_stale"
        elif not controller_active:
            franka_error_detected = True
            error_reason = f"controller_{controller_state}"

        row = {
            "t": elapsed,
            "dt": dt,
            "phase": phase,
            "publish_count": self.publish_count,
            "publish_rate_hz_observed": 1.0 / dt if dt > 0 else NAN,
            "x_ref": target_xyz[0],
            "y_ref": target_xyz[1],
            "z_ref": target_xyz[2],
            "vx_ref": ref_v[0],
            "vy_ref": ref_v[1],
            "vz_ref": ref_v[2],
            "ax_ref": ref_a[0],
            "ay_ref": ref_a[1],
            "az_ref": ref_a[2],
            "jx_ref": ref_j[0],
            "jy_ref": ref_j[1],
            "jz_ref": ref_j[2],
            "target_x": target_xyz[0],
            "target_y": target_xyz[1],
            "target_z": target_xyz[2],
            "target_qx": target_q[0],
            "target_qy": target_q[1],
            "target_qz": target_q[2],
            "target_qw": target_q[3],
            "target_delta_x": target_delta[0],
            "target_delta_y": target_delta[1],
            "target_delta_z": target_delta[2],
            "target_v_backderived_x": target_vx,
            "target_a_backderived_x": target_ax,
            "target_j_backderived_x": target_jx,
            "controller_internal_command_x": internal_xyz[0],
            "controller_internal_command_y": internal_xyz[1],
            "controller_internal_command_z": internal_xyz[2],
            "controller_internal_v_backderived_x": internal_vx,
            "controller_internal_a_backderived_x": internal_ax,
            "controller_internal_j_backderived_x": internal_jx,
            "controller_accepted_target_x": accepted_xyz[0],
            "controller_accepted_target_y": accepted_xyz[1],
            "controller_accepted_target_z": accepted_xyz[2],
            "controller_rt_target_x": rt_xyz[0],
            "controller_rt_target_y": rt_xyz[1],
            "controller_rt_target_z": rt_xyz[2],
            "controller_accept_targets": self.status.get("accept_targets", "UNKNOWN"),
            "target_accepted_count": as_int(self.status, "target_accepted_count"),
            "target_rejected_count": as_int(self.status, "target_rejected_count"),
            "last_target_reject_reason": self.status.get("last_target_reject_reason", "UNKNOWN"),
            "controller_update_period_s": as_float(self.status, "controller_update_period_s"),
            "controller_update_overrun_count": as_int(self.status, "controller_update_overrun_count"),
            "controller_status_age_s": status_age,
            "measured_x": measured_xyz[0],
            "measured_y": measured_xyz[1],
            "measured_z": measured_xyz[2],
            "measured_qx": measured_q[0],
            "measured_qy": measured_q[1],
            "measured_qz": measured_q[2],
            "measured_qw": measured_q[3],
            "measured_pose_age_s": self.measured.age(),
            "robot_state_fresh": self.measured.fresh(self.max_pose_age_s),
            "measured_pose_update_count": self.measured.update_count,
            "target_minus_internal_x": tmi[0],
            "target_minus_internal_y": tmi[1],
            "target_minus_internal_z": tmi[2],
            "target_minus_internal_norm": norm3(tmi) if finite_xyz(tmi) else NAN,
            "internal_minus_measured_x": imm[0],
            "internal_minus_measured_y": imm[1],
            "internal_minus_measured_z": imm[2],
            "internal_minus_measured_norm": norm3(imm) if finite_xyz(imm) else NAN,
            "target_minus_measured_x": tmm[0],
            "target_minus_measured_y": tmm[1],
            "target_minus_measured_z": tmm[2],
            "target_minus_measured_norm": norm3(tmm) if finite_xyz(tmm) else NAN,
            "target_quat_norm": quat_norm(target_q),
            "measured_quat_norm": quat_norm(measured_q) if all(math.isfinite(v) for v in measured_q) else NAN,
            "target_quat_dot_prev": target_quat_dot_prev,
            "orientation_error_angle_rad": quat_angle_error(target_q, measured_q) if all(math.isfinite(v) for v in measured_q) else NAN,
            "target_topic_publisher_count": self.count_publishers(self.target_topic),
            "target_topic_subscriber_count": self.count_subscribers(self.target_topic),
            "controller_active": controller_active,
            "controller_manager_available": controller_manager_available,
            "franka_error_detected": franka_error_detected,
            "error_reason": error_reason,
        }
        for field in STATUS_FLOAT_FIELDS:
            row[field] = as_float(self.status, field)
        for field in STATUS_BOOL_FIELDS:
            row[field] = self.status.get(field, "UNKNOWN")
        return row


def main() -> None:
    rclpy.init()
    node: Optional[PresetTrajectoryNode] = None
    try:
        node = PresetTrajectoryNode()
        code = node.run_test()
    except KeyboardInterrupt:
        code = 130
    except Exception as exc:
        print(f"test1_preset_cartesian_trajectory_node failed: {exc}", file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
