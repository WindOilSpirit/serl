#!/usr/bin/env python3
"""Minimal fixed-offset target test for the SERL Cartesian impedance controller."""

from __future__ import annotations

import argparse
import csv
import math
import re
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


STATUS_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^=\s]+)")
FORCE_FIELDS = [f"cartesian_force_{axis}" for axis in ("x", "y", "z")]
CARTESIAN_TORQUE_FIELDS = [f"cartesian_torque_{axis}" for axis in ("x", "y", "z")]
DESIRED_WRENCH_FIELDS = [
    *(f"desired_wrench_{axis}" for axis in ("x", "y", "z")),
    *(f"desired_wrench_torque_{axis}" for axis in ("x", "y", "z")),
]
WRENCH_EST_FIELDS = [
    *(f"wrench_est_{axis}" for axis in ("x", "y", "z")),
    *(f"wrench_est_torque_{axis}" for axis in ("x", "y", "z")),
]
WRENCH_EST_ERROR_FIELDS = [
    *(f"wrench_est_error_{axis}" for axis in ("x", "y", "z")),
    *(f"wrench_est_error_torque_{axis}" for axis in ("x", "y", "z")),
]
TAU_TASK_FIELDS = [f"tau_task_{index}" for index in range(1, 8)]
TAU_NULLSPACE_FIELDS = [f"tau_nullspace_{index}" for index in range(1, 8)]
CORIOLIS_FIELDS = [f"coriolis_{index}" for index in range(1, 8)]
TAU_BEFORE_FIELDS = [f"tau_before_saturation_{index}" for index in range(1, 8)]
TAU_AFTER_FIELDS = [f"tau_after_saturation_{index}" for index in range(1, 8)]
TAU_FIELDS = [f"tau_command_{index}" for index in range(1, 8)]
Q_FIELDS = [f"q_{index}" for index in range(1, 8)]
DQ_FIELDS = [f"dq_{index}" for index in range(1, 8)]
O_T_EE_FIELDS = [f"O_T_EE_{index}" for index in range(16)]
ZERO_JACOBIAN_FIELDS = [f"zero_jacobian_{index}" for index in range(42)]
TRANSLATIONAL_CLIP_FIELDS = [
    f"translational_clip_{bound}_{axis}"
    for bound in ("min", "max")
    for axis in ("x", "y", "z")
]
ROTATIONAL_CLIP_FIELDS = [
    f"rotational_clip_{bound}_{axis}" for bound in ("min", "max") for axis in ("x", "y", "z")
]
JACOBIAN_VELOCITY_FIELDS = [
    f"cartesian_velocity_from_jacobian_{axis}" for axis in ("x", "y", "z")
]
CONTROLLER_JACOBIAN_VELOCITY_FIELDS = [f"jacobian_velocity_{axis}" for axis in ("x", "y", "z")]
CONTROLLER_JACOBIAN_ANGULAR_VELOCITY_FIELDS = [
    f"jacobian_velocity_angular_{axis}" for axis in ("x", "y", "z")
]
POSE_DIFF_VELOCITY_FIELDS = [
    f"measured_velocity_from_pose_diff_{axis}" for axis in ("x", "y", "z")
]
CONTROLLER_POSE_DIFF_VELOCITY_FIELDS = [f"pose_diff_velocity_{axis}" for axis in ("x", "y", "z")]
CSV_FIELDS = [
    "time_unix_s",
    "time_since_start_s",
    "controller_t",
    "controller_dt",
    "controller_stamp_sec",
    "phase",
    "target_x",
    "measured_x",
    "limited_reference_x",
    "position_error_x",
    *FORCE_FIELDS,
    *CARTESIAN_TORQUE_FIELDS,
    *DESIRED_WRENCH_FIELDS,
    *WRENCH_EST_FIELDS,
    *WRENCH_EST_ERROR_FIELDS,
    "wrench_est_error_norm",
    *TAU_TASK_FIELDS,
    *TAU_NULLSPACE_FIELDS,
    "dot_tau_task_tau_nullspace",
    *CORIOLIS_FIELDS,
    *TAU_BEFORE_FIELDS,
    *TAU_AFTER_FIELDS,
    *TAU_FIELDS,
    *Q_FIELDS,
    *DQ_FIELDS,
    *O_T_EE_FIELDS,
    *ZERO_JACOBIAN_FIELDS,
    *CONTROLLER_JACOBIAN_VELOCITY_FIELDS,
    *CONTROLLER_JACOBIAN_ANGULAR_VELOCITY_FIELDS,
    *JACOBIAN_VELOCITY_FIELDS,
    *CONTROLLER_POSE_DIFF_VELOCITY_FIELDS,
    *POSE_DIFF_VELOCITY_FIELDS,
    "velocity_direction_cosine",
    "velocity_norm_ratio",
    "velocity_diff_norm",
    "commanded_torque_norm",
    "tau_rate_limited",
    "torque_rate_limited",
    "reference_clipped",
    "use_robot_state_q_dq",
    "enable_nullspace_torque",
    "reference_limit_mode",
    "translational_stiffness",
    "translational_damping",
    "filter_coeff",
    *TRANSLATIONAL_CLIP_FIELDS,
    *ROTATIONAL_CLIP_FIELDS,
]


@dataclass
class PoseSnapshot:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def pose_from_msg(msg: PoseStamped) -> PoseSnapshot:
    pose = msg.pose
    return PoseSnapshot(
        x=float(pose.position.x),
        y=float(pose.position.y),
        z=float(pose.position.z),
        qx=float(pose.orientation.x),
        qy=float(pose.orientation.y),
        qz=float(pose.orientation.z),
        qw=float(pose.orientation.w),
    )


def make_pose_msg(node: Node, pose: PoseSnapshot, target_x: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base"
    msg.pose.position.x = target_x
    msg.pose.position.y = pose.y
    msg.pose.position.z = pose.z
    msg.pose.orientation.x = pose.qx
    msg.pose.orientation.y = pose.qy
    msg.pose.orientation.z = pose.qz
    msg.pose.orientation.w = pose.qw
    return msg


def parse_status(text: str) -> dict[str, str]:
    return {match.group(1): match.group(2) for match in STATUS_RE.finditer(text)}


def as_float(value: str | None) -> float:
    if value is None:
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def fmt_csv_float(value: float) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.12f}"


class FixedOffsetTest(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("test_serl_impedance_fixed_offset")
        self.args = args
        self.measured_pose: PoseSnapshot | None = None
        self.limited_reference_pose: PoseSnapshot | None = None
        self.status: dict[str, str] = {}

        self.target_pub = self.create_publisher(PoseStamped, args.target_pose_topic, 10)
        self.create_subscription(PoseStamped, args.measured_pose_topic, self._measured_cb, 10)
        self.create_subscription(
            PoseStamped, args.limited_reference_topic, self._limited_reference_cb, 10
        )
        self.create_subscription(String, args.controller_status_topic, self._status_cb, 10)

        service_name = args.controller_manager.rstrip("/") + "/list_controllers"
        if not service_name.startswith("/"):
            service_name = "/" + service_name
        self.controller_client = self.create_client(ListControllers, service_name)

    def _measured_cb(self, msg: PoseStamped) -> None:
        self.measured_pose = pose_from_msg(msg)

    def _limited_reference_cb(self, msg: PoseStamped) -> None:
        self.limited_reference_pose = pose_from_msg(msg)

    def _status_cb(self, msg: String) -> None:
        self.status = parse_status(msg.data)

    def wait_for_controller_active(self) -> None:
        deadline = time.monotonic() + self.args.timeout_s
        self.get_logger().info(f"Waiting for controller active: {self.args.controller_name}")
        while time.monotonic() < deadline:
            if not self.controller_client.wait_for_service(timeout_sec=1.0):
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            future = self.controller_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            response = future.result()
            if response is None:
                continue
            for controller in response.controller:
                if (
                    controller.name == self.args.controller_name
                    and controller.state.lower() == "active"
                ):
                    self.get_logger().info(f"Controller is active: {controller.name}")
                    return
            time.sleep(0.2)
        raise TimeoutError(f"Controller did not become active within {self.args.timeout_s:.1f}s")

    def wait_for_measured_pose(self) -> PoseSnapshot:
        deadline = time.monotonic() + self.args.timeout_s
        self.get_logger().info(f"Waiting for measured pose: {self.args.measured_pose_topic}")
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.measured_pose is not None:
                self.get_logger().info(
                    "Measured pose received: "
                    f"x={self.measured_pose.x:.6f} "
                    f"y={self.measured_pose.y:.6f} "
                    f"z={self.measured_pose.z:.6f}"
                )
                return self.measured_pose
        raise TimeoutError(f"No measured pose received within {self.args.timeout_s:.1f}s")

    def wait_for_target_subscriber(self) -> None:
        deadline = time.monotonic() + min(self.args.timeout_s, 10.0)
        self.get_logger().info(f"Waiting for target subscriber: {self.args.target_pose_topic}")
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.target_pub.get_subscription_count() > 0:
                self.get_logger().info("Target subscriber is connected.")
                return
        raise TimeoutError("No subscriber connected to target pose topic.")

    def run_phase(
        self,
        writer: csv.DictWriter,
        csv_file,
        phase: str,
        initial_pose: PoseSnapshot,
        target_x: float,
        duration_s: float,
        start_mono: float,
        rows: list[dict[str, str]],
    ) -> None:
        end_time = time.monotonic() + duration_s
        publish_period_s = 1.0 / self.args.publish_rate_hz
        sample_period_s = 1.0 / self.args.sample_rate_hz
        next_publish = 0.0
        next_sample = 0.0

        self.get_logger().info(
            f"Phase {phase}: target_x={target_x:.6f}, duration={duration_s:.2f}s"
        )
        while rclpy.ok() and time.monotonic() < end_time:
            now = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.001)

            if now >= next_publish:
                self.target_pub.publish(make_pose_msg(self, initial_pose, target_x))
                next_publish = now + publish_period_s

            if now >= next_sample:
                row = self.make_row(phase, target_x, start_mono)
                writer.writerow(row)
                rows.append(row)
                if len(rows) % 25 == 0:
                    csv_file.flush()
                next_sample = now + sample_period_s

        csv_file.flush()

    def make_row(self, phase: str, target_x: float, start_mono: float) -> dict[str, str]:
        measured_x = self.measured_pose.x if self.measured_pose is not None else math.nan
        limited_x = (
            self.limited_reference_pose.x
            if self.limited_reference_pose is not None
            else as_float(self.status.get("limited_x"))
        )
        row = {
            "time_unix_s": fmt_csv_float(time.time()),
            "time_since_start_s": fmt_csv_float(time.monotonic() - start_mono),
            "controller_t": fmt_csv_float(as_float(self.status.get("t"))),
            "controller_dt": fmt_csv_float(as_float(self.status.get("dt"))),
            "controller_stamp_sec": fmt_csv_float(as_float(self.status.get("stamp_sec"))),
            "phase": phase,
            "target_x": fmt_csv_float(target_x),
            "measured_x": fmt_csv_float(measured_x),
            "limited_reference_x": fmt_csv_float(limited_x),
            "position_error_x": fmt_csv_float(as_float(self.status.get("position_error_x"))),
            "commanded_torque_norm": fmt_csv_float(
                as_float(self.status.get("commanded_torque_norm"))
            ),
            "tau_rate_limited": self.status.get("tau_rate_limited", ""),
            "torque_rate_limited": self.status.get("torque_rate_limited", ""),
            "reference_clipped": self.status.get("reference_clipped", ""),
            "use_robot_state_q_dq": self.status.get("use_robot_state_q_dq", ""),
            "enable_nullspace_torque": self.status.get("enable_nullspace_torque", ""),
            "reference_limit_mode": self.status.get("reference_limit_mode", ""),
            "translational_stiffness": fmt_csv_float(
                as_float(self.status.get("translational_stiffness"))
            ),
            "translational_damping": fmt_csv_float(
                as_float(self.status.get("translational_damping"))
            ),
            "filter_coeff": fmt_csv_float(as_float(self.status.get("filter_coeff"))),
            "velocity_direction_cosine": fmt_csv_float(
                as_float(self.status.get("velocity_direction_cosine"))
            ),
            "velocity_norm_ratio": fmt_csv_float(as_float(self.status.get("velocity_norm_ratio"))),
            "velocity_diff_norm": fmt_csv_float(as_float(self.status.get("velocity_diff_norm"))),
        }
        for field in (
            FORCE_FIELDS
            + CARTESIAN_TORQUE_FIELDS
            + DESIRED_WRENCH_FIELDS
            + WRENCH_EST_FIELDS
            + WRENCH_EST_ERROR_FIELDS
            + ["wrench_est_error_norm"]
            + TAU_TASK_FIELDS
            + TAU_NULLSPACE_FIELDS
            + ["dot_tau_task_tau_nullspace"]
            + CORIOLIS_FIELDS
            + TAU_BEFORE_FIELDS
            + TAU_AFTER_FIELDS
            + TAU_FIELDS
            + Q_FIELDS
            + DQ_FIELDS
            + O_T_EE_FIELDS
            + ZERO_JACOBIAN_FIELDS
            + CONTROLLER_JACOBIAN_VELOCITY_FIELDS
            + CONTROLLER_JACOBIAN_ANGULAR_VELOCITY_FIELDS
            + JACOBIAN_VELOCITY_FIELDS
            + CONTROLLER_POSE_DIFF_VELOCITY_FIELDS
            + POSE_DIFF_VELOCITY_FIELDS
            + TRANSLATIONAL_CLIP_FIELDS
            + ROTATIONAL_CLIP_FIELDS
        ):
            row[field] = fmt_csv_float(as_float(self.status.get(field)))
        return row


def default_output_csv() -> Path:
    package_dir = Path(__file__).resolve().parents[2]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return package_dir / "debug_output" / f"test_serl_impedance_fixed_offset_{stamp}.csv"


def finite_float_from_row(row: dict[str, str], field: str) -> float:
    return as_float(row.get(field))


def summarize(rows: list[dict[str, str]], offset_m: float) -> list[str]:
    offset_rows = [row for row in rows if row.get("phase") == "offset_plus_x"]
    if not offset_rows:
        return ["No offset phase rows were recorded."]

    first = offset_rows[0]
    last = offset_rows[-1]
    measured_delta = finite_float_from_row(last, "measured_x") - finite_float_from_row(
        first, "measured_x"
    )
    target_x = finite_float_from_row(last, "target_x")
    limited_x = finite_float_from_row(last, "limited_reference_x")
    limited_error = limited_x - target_x

    lines = [
        f"offset target: +{offset_m * 1000.0:.3f} mm",
        f"measured_x delta during offset phase: {measured_delta * 1000.0:.6f} mm",
        f"final limited_reference_x - target_x: {limited_error * 1000.0:.6f} mm",
        f"final cartesian_force_x: {finite_float_from_row(last, 'cartesian_force_x'):.6f} N",
        f"final commanded_torque_norm: {finite_float_from_row(last, 'commanded_torque_norm'):.6f} Nm",
    ]

    if abs(measured_delta) < 5.0e-6:
        lines.append("judgement: measured_x was effectively still.")
    elif measured_delta < 0.0:
        lines.append("judgement: measured_x moved in the opposite direction.")
    elif measured_delta < 0.2 * offset_m:
        lines.append("judgement: measured_x moved in +x, but the response was small/slow.")
    else:
        lines.append("judgement: measured_x moved in +x.")

    if math.isfinite(limited_error) and abs(limited_error) > 5.0e-5:
        lines.append("judgement: limited_reference_x did not converge to target_x.")
    else:
        lines.append("judgement: limited_reference_x matched target_x within 0.05 mm.")
    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a minimal +1 mm fixed-offset target test without SpaceMouse."
    )
    parser.add_argument("--controller-name", default="serl_cartesian_impedance_controller")
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument(
        "--target-pose-topic",
        default="/serl_cartesian_impedance_controller/target_pose",
    )
    parser.add_argument(
        "--measured-pose-topic",
        default="/serl_cartesian_impedance_controller/debug/measured_pose",
    )
    parser.add_argument(
        "--limited-reference-topic",
        default="/serl_cartesian_impedance_controller/debug/clipped_target_pose",
    )
    parser.add_argument(
        "--controller-status-topic",
        default="/serl_cartesian_impedance_controller/debug/status",
    )
    parser.add_argument("--offset-m", type=float, default=0.001)
    parser.add_argument("--initial-hold-s", type=float, default=2.0)
    parser.add_argument("--offset-hold-s", type=float, default=5.0)
    parser.add_argument("--return-hold-s", type=float, default=2.0)
    parser.add_argument("--publish-rate-hz", type=float, default=100.0)
    parser.add_argument("--sample-rate-hz", type=float, default=50.0)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--output-csv", type=Path, default=default_output_csv())
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = FixedOffsetTest(args)
    rows: list[dict[str, str]] = []
    try:
        node.wait_for_controller_active()
        initial_pose = node.wait_for_measured_pose()
        node.wait_for_target_subscriber()

        start_mono = time.monotonic()
        with args.output_csv.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            node.run_phase(
                writer,
                csv_file,
                "hold_measured_before",
                initial_pose,
                initial_pose.x,
                args.initial_hold_s,
                start_mono,
                rows,
            )
            node.run_phase(
                writer,
                csv_file,
                "offset_plus_x",
                initial_pose,
                initial_pose.x + args.offset_m,
                args.offset_hold_s,
                start_mono,
                rows,
            )
            node.run_phase(
                writer,
                csv_file,
                "hold_measured_after",
                initial_pose,
                initial_pose.x,
                args.return_hold_s,
                start_mono,
                rows,
            )

        node.get_logger().info(f"CSV saved: {args.output_csv}")
        for line in summarize(rows, args.offset_m):
            node.get_logger().info(line)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
