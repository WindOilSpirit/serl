#!/usr/bin/env python3
"""Record the minimal joint sine effort controller response."""

from __future__ import annotations

import argparse
import csv
import math
import re
import time
from datetime import datetime
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from std_msgs.msg import String


STATUS_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^=\s]+)")
CSV_FIELDS = [
    "time_unix_s",
    "time_since_start_s",
    "controller_t",
    "elapsed_s",
    "dt",
    "joint_number",
    "q_4",
    "dq_4",
    "tau_command_4",
    "amplitude_nm",
    "frequency_hz",
    "start_delay_s",
    "duration_s",
    "command_finished",
]


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


class JointSineTorqueRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("test_joint_sine_torque")
        self.args = args
        self.status: dict[str, str] = {}
        self.create_subscription(String, args.status_topic, self._status_cb, 10)

        service_name = args.controller_manager.rstrip("/") + "/list_controllers"
        if not service_name.startswith("/"):
            service_name = "/" + service_name
        self.controller_client = self.create_client(ListControllers, service_name)

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

    def wait_for_status(self) -> None:
        deadline = time.monotonic() + self.args.timeout_s
        self.get_logger().info(f"Waiting for status: {self.args.status_topic}")
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.status:
                return
        raise TimeoutError(f"No status received within {self.args.timeout_s:.1f}s")

    def make_row(self, start_mono: float) -> dict[str, str]:
        return {
            "time_unix_s": fmt_csv_float(time.time()),
            "time_since_start_s": fmt_csv_float(time.monotonic() - start_mono),
            "controller_t": fmt_csv_float(as_float(self.status.get("t"))),
            "elapsed_s": fmt_csv_float(as_float(self.status.get("elapsed_s"))),
            "dt": fmt_csv_float(as_float(self.status.get("dt"))),
            "joint_number": self.status.get("joint_number", ""),
            "q_4": fmt_csv_float(as_float(self.status.get("q_4"))),
            "dq_4": fmt_csv_float(as_float(self.status.get("dq_4"))),
            "tau_command_4": fmt_csv_float(as_float(self.status.get("tau_command_4"))),
            "amplitude_nm": fmt_csv_float(as_float(self.status.get("amplitude_nm"))),
            "frequency_hz": fmt_csv_float(as_float(self.status.get("frequency_hz"))),
            "start_delay_s": fmt_csv_float(as_float(self.status.get("start_delay_s"))),
            "duration_s": fmt_csv_float(as_float(self.status.get("duration_s"))),
            "command_finished": self.status.get("command_finished", ""),
        }

    def record(self) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        start_mono = time.monotonic()
        end_mono = start_mono + self.args.record_s
        sample_period_s = 1.0 / self.args.sample_rate_hz
        next_sample = 0.0

        with self.args.output_csv.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            while rclpy.ok() and time.monotonic() < end_mono:
                now = time.monotonic()
                rclpy.spin_once(self, timeout_sec=0.001)
                if now >= next_sample:
                    row = self.make_row(start_mono)
                    writer.writerow(row)
                    rows.append(row)
                    next_sample = now + sample_period_s
            csv_file.flush()
        return rows


def default_output_csv() -> Path:
    package_dir = Path(__file__).resolve().parents[2]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return package_dir / "debug_output" / f"joint_sine_torque_{stamp}.csv"


def summarize(rows: list[dict[str, str]]) -> list[str]:
    q_values = [as_float(row.get("q_4")) for row in rows]
    dq_values = [as_float(row.get("dq_4")) for row in rows]
    tau_values = [as_float(row.get("tau_command_4")) for row in rows]
    q_values = [value for value in q_values if math.isfinite(value)]
    dq_values = [value for value in dq_values if math.isfinite(value)]
    tau_values = [value for value in tau_values if math.isfinite(value)]
    if not q_values:
        return ["No finite q_4 samples were recorded."]
    q_range = max(q_values) - min(q_values)
    dq_range = max(dq_values) - min(dq_values) if dq_values else math.nan
    tau_range = max(tau_values) - min(tau_values) if tau_values else math.nan
    lines = [
        f"q_4 range: {q_range:.9f} rad",
        f"dq_4 range: {dq_range:.9f} rad/s",
        f"tau_command_4 range: {tau_range:.9f} Nm",
    ]
    if q_range < 1.0e-5 and (not dq_values or dq_range < 1.0e-4):
        lines.append("judgement: q_4/dq_4 response is effectively still at this torque level.")
    else:
        lines.append("judgement: q_4/dq_4 shows an observable response.")
    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record joint sine torque debug status to CSV.")
    parser.add_argument("--controller-name", default="joint_sine_torque_test_controller")
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument(
        "--status-topic",
        default="/joint_sine_torque_test_controller/debug/status",
    )
    parser.add_argument("--record-s", type=float, default=4.0)
    parser.add_argument("--sample-rate-hz", type=float, default=100.0)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--output-csv", type=Path, default=default_output_csv())
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = JointSineTorqueRecorder(args)
    try:
        node.wait_for_controller_active()
        node.wait_for_status()
        rows = node.record()
        node.get_logger().info(f"CSV saved: {args.output_csv}")
        for line in summarize(rows):
            node.get_logger().info(line)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
