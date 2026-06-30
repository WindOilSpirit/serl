#!/usr/bin/env python3
"""Axis response matrix test for the SERL Cartesian impedance controller.

This script publishes small fixed Cartesian target offsets without using the
SpaceMouse. Run only with the robot clear of contact, watched, and ready to
stop. It is intended to identify whether x/y/z coupling appears in the target,
the controller wrench, or the measured robot response.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped

from test_serl_impedance_fixed_offset import (
    CARTESIAN_TORQUE_FIELDS,
    CORIOLIS_FIELDS,
    DESIRED_WRENCH_FIELDS,
    DQ_FIELDS,
    FORCE_FIELDS,
    O_T_EE_FIELDS,
    POSE_DIFF_VELOCITY_FIELDS,
    Q_FIELDS,
    TAU_AFTER_FIELDS,
    TAU_BEFORE_FIELDS,
    TAU_FIELDS,
    TAU_NULLSPACE_FIELDS,
    TAU_TASK_FIELDS,
    WRENCH_EST_ERROR_FIELDS,
    WRENCH_EST_FIELDS,
    ZERO_JACOBIAN_FIELDS,
    FixedOffsetTest,
    PoseSnapshot,
    as_float,
    fmt_csv_float,
)


AXIS_NAMES = ("x", "y", "z")
BASE_FIELDS = [
    "time_unix_s",
    "time_since_start_s",
    "controller_t",
    "controller_dt",
    "phase",
    "command_axis",
    "command_offset_m",
    *(f"target_{axis}" for axis in AXIS_NAMES),
    *(f"measured_{axis}" for axis in AXIS_NAMES),
    *(f"limited_reference_{axis}" for axis in AXIS_NAMES),
    *(f"target_delta_from_initial_{axis}" for axis in AXIS_NAMES),
    *(f"measured_delta_from_initial_{axis}" for axis in AXIS_NAMES),
    *(f"position_error_{axis}" for axis in AXIS_NAMES),
    "commanded_torque_norm",
    "tau_rate_limited",
    "torque_rate_limited",
    "reference_clipped",
    "control_law_mode",
    "translational_stiffness",
    "translational_damping",
    "rotational_stiffness",
    "rotational_damping",
    "velocity_direction_cosine",
    "velocity_norm_ratio",
    "velocity_diff_norm",
]
CSV_FIELDS = [
    *BASE_FIELDS,
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
    *(f"tau_J_{index}" for index in range(1, 8)),
    *(f"tau_J_d_{index}" for index in range(1, 8)),
    *(f"tau_ext_hat_filtered_{index}" for index in range(1, 8)),
    *(f"O_F_ext_hat_K_{index}" for index in range(6)),
    *(f"K_F_ext_hat_K_{index}" for index in range(6)),
    *Q_FIELDS,
    *DQ_FIELDS,
    *O_T_EE_FIELDS,
    *ZERO_JACOBIAN_FIELDS,
    *POSE_DIFF_VELOCITY_FIELDS,
]


def default_output_csv() -> Path:
    package_dir = Path(__file__).resolve().parents[2]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return package_dir / "debug_output" / f"test_serl_impedance_axis_response_{stamp}.csv"


def pose_position(pose: PoseSnapshot) -> np.ndarray:
    return np.array([pose.x, pose.y, pose.z], dtype=np.float64)


def make_pose_msg_from_position(
    node: FixedOffsetTest, initial_pose: PoseSnapshot, target_position: np.ndarray
) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base"
    msg.pose.position.x = float(target_position[0])
    msg.pose.position.y = float(target_position[1])
    msg.pose.position.z = float(target_position[2])
    msg.pose.orientation.x = initial_pose.qx
    msg.pose.orientation.y = initial_pose.qy
    msg.pose.orientation.z = initial_pose.qz
    msg.pose.orientation.w = initial_pose.qw
    return msg


def parse_axis_token(token: str) -> tuple[str, float]:
    value = token.strip().lower()
    sign = 1.0
    if value.startswith("+"):
        value = value[1:]
    elif value.startswith("-"):
        sign = -1.0
        value = value[1:]
    if value not in AXIS_NAMES:
        raise argparse.ArgumentTypeError(f"axis token must be one of +/-x,+/-y,+/-z, got {token!r}")
    return value, sign


class AxisResponseTest(FixedOffsetTest):
    def run_vector_phase(
        self,
        writer: csv.DictWriter,
        csv_file,
        phase: str,
        command_axis: str,
        command_offset_m: float,
        initial_pose: PoseSnapshot,
        target_position: np.ndarray,
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
            f"Phase {phase}: target=({target_position[0]:.6f}, "
            f"{target_position[1]:.6f}, {target_position[2]:.6f}), "
            f"duration={duration_s:.2f}s"
        )
        while rclpy.ok() and time.monotonic() < end_time:
            now = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.001)
            if now >= next_publish:
                self.target_pub.publish(make_pose_msg_from_position(self, initial_pose, target_position))
                next_publish = now + publish_period_s
            if now >= next_sample:
                row = self.make_axis_row(
                    phase,
                    command_axis,
                    command_offset_m,
                    initial_pose,
                    target_position,
                    start_mono,
                )
                writer.writerow(row)
                rows.append(row)
                if len(rows) % 25 == 0:
                    csv_file.flush()
                next_sample = now + sample_period_s
        csv_file.flush()

    def make_axis_row(
        self,
        phase: str,
        command_axis: str,
        command_offset_m: float,
        initial_pose: PoseSnapshot,
        target_position: np.ndarray,
        start_mono: float,
    ) -> dict[str, str]:
        initial_position = pose_position(initial_pose)
        measured_position = (
            pose_position(self.measured_pose)
            if self.measured_pose is not None
            else np.full(3, math.nan, dtype=np.float64)
        )
        limited_position = (
            pose_position(self.limited_reference_pose)
            if self.limited_reference_pose is not None
            else np.array(
                [as_float(self.status.get(f"limited_{axis}")) for axis in AXIS_NAMES],
                dtype=np.float64,
            )
        )
        row = {
            "time_unix_s": fmt_csv_float(time.time()),
            "time_since_start_s": fmt_csv_float(time.monotonic() - start_mono),
            "controller_t": fmt_csv_float(as_float(self.status.get("t"))),
            "controller_dt": fmt_csv_float(as_float(self.status.get("dt"))),
            "phase": phase,
            "command_axis": command_axis,
            "command_offset_m": fmt_csv_float(command_offset_m),
            "commanded_torque_norm": fmt_csv_float(as_float(self.status.get("commanded_torque_norm"))),
            "tau_rate_limited": self.status.get("tau_rate_limited", ""),
            "torque_rate_limited": self.status.get("torque_rate_limited", ""),
            "reference_clipped": self.status.get("reference_clipped", ""),
            "control_law_mode": self.status.get("control_law_mode", ""),
            "translational_stiffness": fmt_csv_float(
                as_float(self.status.get("translational_stiffness"))
            ),
            "translational_damping": fmt_csv_float(as_float(self.status.get("translational_damping"))),
            "rotational_stiffness": fmt_csv_float(as_float(self.status.get("rotational_stiffness"))),
            "rotational_damping": fmt_csv_float(as_float(self.status.get("rotational_damping"))),
            "velocity_direction_cosine": fmt_csv_float(
                as_float(self.status.get("velocity_direction_cosine"))
            ),
            "velocity_norm_ratio": fmt_csv_float(as_float(self.status.get("velocity_norm_ratio"))),
            "velocity_diff_norm": fmt_csv_float(as_float(self.status.get("velocity_diff_norm"))),
        }
        for index, axis in enumerate(AXIS_NAMES):
            row[f"target_{axis}"] = fmt_csv_float(float(target_position[index]))
            row[f"measured_{axis}"] = fmt_csv_float(float(measured_position[index]))
            row[f"limited_reference_{axis}"] = fmt_csv_float(float(limited_position[index]))
            row[f"target_delta_from_initial_{axis}"] = fmt_csv_float(
                float(target_position[index] - initial_position[index])
            )
            row[f"measured_delta_from_initial_{axis}"] = fmt_csv_float(
                float(measured_position[index] - initial_position[index])
            )
            row[f"position_error_{axis}"] = fmt_csv_float(
                as_float(self.status.get(f"position_error_{axis}"))
            )
        for field in CSV_FIELDS:
            row.setdefault(field, fmt_csv_float(as_float(self.status.get(field))))
        return row


def finite(row: dict[str, str], field: str) -> float:
    return as_float(row.get(field))


def summarize(rows: list[dict[str, str]]) -> list[str]:
    lines: list[str] = []
    phases = [row["phase"] for row in rows if row["phase"].startswith("offset_")]
    for phase in dict.fromkeys(phases):
        phase_rows = [row for row in rows if row.get("phase") == phase]
        if len(phase_rows) < 2:
            continue
        first = phase_rows[0]
        last = phase_rows[-1]
        measured_delta = [
            (finite(last, f"measured_{axis}") - finite(first, f"measured_{axis}")) * 1000.0
            for axis in AXIS_NAMES
        ]
        target_delta = [
            (finite(last, f"target_{axis}") - finite(first, f"target_{axis}")) * 1000.0
            for axis in AXIS_NAMES
        ]
        force = [finite(last, f"cartesian_force_{axis}") for axis in AXIS_NAMES]
        lines.append(
            f"{phase}: target_delta_mm={target_delta}, "
            f"measured_delta_mm={measured_delta}, final_force_N={force}"
        )
    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run +/-x,+/-y,+/-z fixed-offset axis response tests without SpaceMouse."
    )
    parser.add_argument("--controller-name", default="serl_cartesian_impedance_controller")
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--target-pose-topic", default="/serl_cartesian_impedance_controller/target_pose")
    parser.add_argument(
        "--measured-pose-topic", default="/serl_cartesian_impedance_controller/debug/measured_pose"
    )
    parser.add_argument(
        "--limited-reference-topic",
        default="/serl_cartesian_impedance_controller/debug/clipped_target_pose",
    )
    parser.add_argument(
        "--controller-status-topic", default="/serl_cartesian_impedance_controller/debug/status"
    )
    parser.add_argument("--offset-m", type=float, default=0.001)
    parser.add_argument("--axes", nargs="+", type=parse_axis_token, default=["+x", "-x", "+y", "-y", "+z", "-z"])
    parser.add_argument("--initial-hold-s", type=float, default=1.5)
    parser.add_argument("--offset-hold-s", type=float, default=3.0)
    parser.add_argument("--neutral-hold-s", type=float, default=1.0)
    parser.add_argument("--publish-rate-hz", type=float, default=100.0)
    parser.add_argument("--sample-rate-hz", type=float, default=50.0)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--output-csv", type=Path, default=default_output_csv())
    args = parser.parse_args()
    normalized_axes = []
    for item in args.axes:
        normalized_axes.append(parse_axis_token(item) if isinstance(item, str) else item)
    args.axes = normalized_axes
    return args


def main() -> None:
    args = parse_args()
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = AxisResponseTest(args)
    rows: list[dict[str, str]] = []
    try:
        node.wait_for_controller_active()
        initial_pose = node.wait_for_measured_pose()
        node.wait_for_target_subscriber()
        initial_position = pose_position(initial_pose)
        start_mono = time.monotonic()

        with args.output_csv.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS, extrasaction="ignore")
            writer.writeheader()
            node.run_vector_phase(
                writer,
                csv_file,
                "hold_initial",
                "none",
                0.0,
                initial_pose,
                initial_position,
                args.initial_hold_s,
                start_mono,
                rows,
            )
            for axis, sign in args.axes:
                offset = np.zeros(3, dtype=np.float64)
                axis_index = AXIS_NAMES.index(axis)
                offset[axis_index] = sign * args.offset_m
                phase_name = f"offset_{'plus' if sign > 0 else 'minus'}_{axis}"
                node.run_vector_phase(
                    writer,
                    csv_file,
                    phase_name,
                    f"{'+' if sign > 0 else '-'}{axis}",
                    sign * args.offset_m,
                    initial_pose,
                    initial_position + offset,
                    args.offset_hold_s,
                    start_mono,
                    rows,
                )
                node.run_vector_phase(
                    writer,
                    csv_file,
                    f"neutral_after_{axis}",
                    "none",
                    0.0,
                    initial_pose,
                    initial_position,
                    args.neutral_hold_s,
                    start_mono,
                    rows,
                )

        node.get_logger().info(f"CSV saved: {args.output_csv}")
        for line in summarize(rows):
            node.get_logger().info(line)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
