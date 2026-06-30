#!/usr/bin/env python3
"""Dashboard for the SERL-style Cartesian impedance teleop route."""

from __future__ import annotations

import math
import os
import re
import signal
import subprocess
import threading
import time
import tkinter as tk
import csv
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, ttk

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String


PROJECT_DIR = Path(
    os.environ.get(
        "SPACEMOUSE_FRANKA_TELEOP_DIR",
        str(Path(__file__).resolve().parents[1]),
    )
)
DEFAULT_LOG_DIR = Path("/tmp/spacemouse_franka_teleop_logs")
RECORD_PERIOD_S = 0.02
POSE_FIELD_NAMES = ("x", "y", "z", "qx", "qy", "qz", "qw")
CONTROLLER_DEBUG_RECORD_FIELDS = [
    "control_law_mode",
    *(f"position_error_{axis}" for axis in ("x", "y", "z")),
    *(f"cartesian_force_{axis}" for axis in ("x", "y", "z")),
    *(f"cartesian_torque_{axis}" for axis in ("x", "y", "z")),
    *(f"desired_wrench_{axis}" for axis in ("x", "y", "z")),
    *(f"desired_wrench_torque_{axis}" for axis in ("x", "y", "z")),
    *(f"wrench_est_{axis}" for axis in ("x", "y", "z")),
    *(f"wrench_est_torque_{axis}" for axis in ("x", "y", "z")),
    "wrench_est_error_norm",
    *(f"tau_task_{index}" for index in range(1, 8)),
    *(f"tau_command_{index}" for index in range(1, 8)),
]
RECORD_HEADER = [
    "frame",
    "time_unix_s",
    "time_since_start_s",
    "controller_running",
    "controller_exit_code",
    "teleop_running",
    "teleop_exit_code",
    "teleop_state",
    "deadman",
    "fine_mode",
    "spacemouse_linear_x",
    "spacemouse_linear_y",
    "spacemouse_linear_z",
    "spacemouse_angular_x",
    "spacemouse_angular_y",
    "spacemouse_angular_z",
    "action_age_s",
    *[f"measured_{name}" for name in POSE_FIELD_NAMES],
    "measured_pose_age_s",
    *[f"target_{name}" for name in POSE_FIELD_NAMES],
    "target_pose_age_s",
    *[f"controller_raw_target_{name}" for name in POSE_FIELD_NAMES],
    "controller_raw_target_age_s",
    *[f"controller_smoothed_target_{name}" for name in POSE_FIELD_NAMES],
    "controller_smoothed_target_age_s",
    *[f"controller_clipped_target_{name}" for name in POSE_FIELD_NAMES],
    "controller_clipped_target_age_s",
    *[f"controller_measured_{name}" for name in POSE_FIELD_NAMES],
    "controller_measured_age_s",
    "target_minus_measured_x",
    "target_minus_measured_y",
    "target_minus_measured_z",
    "target_measured_error_norm",
    "raw_minus_smoothed_norm",
    "smoothed_minus_clipped_norm",
    "clipped_minus_controller_measured_norm",
    "ros2_controller_active",
    "ros2_controller_state",
    "server_state",
    "server_reason",
    "server_target_publish_count",
    "reference_clipped",
    "target_distance_clamped",
    "desired_speed_limited",
    "desired_acceleration_limited",
    "jerk_limited",
    "step_limited",
    "commanded_torque_norm",
    *CONTROLLER_DEBUG_RECORD_FIELDS,
    "external_force_norm",
    "external_torque_norm",
    "server_status",
    "controller_status",
    "alert",
]


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


def twist_to_array(msg: TwistStamped) -> np.ndarray:
    return np.array(
        [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        ],
        dtype=np.float64,
    )


def wrench_to_array(msg: WrenchStamped) -> np.ndarray:
    return np.array(
        [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ],
        dtype=np.float64,
    )


def parse_status(text: str) -> dict[str, str]:
    pairs: dict[str, str] = {}
    for match in re.finditer(r"([A-Za-z_][A-Za-z0-9_]*)=([^=\s]+)", text):
        pairs[match.group(1)] = match.group(2)
    return pairs


def fmt_float(value: float, digits: int = 6) -> str:
    if not math.isfinite(value):
        return "等待"
    return f"{value:.{digits}f}"


def fmt_mm(value_m: float, digits: int = 3) -> str:
    if not math.isfinite(value_m):
        return "等待"
    return f"{value_m * 1000.0:.{digits}f}"


def fmt_age(last_time: float, now: float, stale_timeout_s: float) -> str:
    if last_time <= 0.0:
        return "未收到"
    age = now - last_time
    state = "新鲜" if age <= stale_timeout_s else "超时"
    return f"{age:.3f}s / {state}"


def fmt_pose(pose: np.ndarray | None) -> str:
    if pose is None:
        return "等待数据"
    return (
        f"x {fmt_mm(pose[0])} mm  y {fmt_mm(pose[1])} mm  z {fmt_mm(pose[2])} mm\n"
        f"qx {fmt_float(pose[3])}  qy {fmt_float(pose[4])}  "
        f"qz {fmt_float(pose[5])}  qw {fmt_float(pose[6])}"
    )


def fmt_vec(values: np.ndarray | None, names: tuple[str, ...]) -> str:
    if values is None:
        return "等待数据"
    return "  ".join(f"{name} {fmt_float(float(value), 5)}" for name, value in zip(names, values))


def pose_error(a: np.ndarray | None, b: np.ndarray | None) -> float:
    if a is None or b is None:
        return math.nan
    delta = a[:3] - b[:3]
    if not np.all(np.isfinite(delta)):
        return math.nan
    return float(np.linalg.norm(delta))


def _float_for_csv(value: float) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.12f}"


def _array_values_for_csv(values: np.ndarray | None, count: int) -> list[str]:
    if values is None:
        return ["nan"] * count
    result: list[str] = []
    for index in range(count):
        if index >= len(values):
            result.append("nan")
        else:
            result.append(_float_for_csv(float(values[index])))
    return result


def _pose_values_for_csv(pose: np.ndarray | None) -> list[str]:
    return _array_values_for_csv(pose, 7)


def _age_for_csv(now: float, last_time: float) -> str:
    if last_time <= 0.0:
        return "nan"
    return _float_for_csv(now - last_time)


@dataclass
class DashboardState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    action: np.ndarray | None = None
    action_time: float = 0.0
    deadman: bool = False
    deadman_time: float = 0.0
    fine_mode: bool = False
    fine_mode_time: float = 0.0
    target_pose: np.ndarray | None = None
    target_pose_time: float = 0.0
    measured_pose: np.ndarray | None = None
    measured_pose_time: float = 0.0
    raw_target_pose: np.ndarray | None = None
    raw_target_pose_time: float = 0.0
    smoothed_target_pose: np.ndarray | None = None
    smoothed_target_pose_time: float = 0.0
    clipped_target_pose: np.ndarray | None = None
    clipped_target_pose_time: float = 0.0
    controller_measured_pose: np.ndarray | None = None
    controller_measured_pose_time: float = 0.0
    external_wrench: np.ndarray | None = None
    external_wrench_time: float = 0.0
    server_status_raw: str = ""
    server_status: dict[str, str] = field(default_factory=dict)
    server_status_time: float = 0.0
    controller_status_raw: str = ""
    controller_status: dict[str, str] = field(default_factory=dict)
    controller_status_time: float = 0.0
    controller_state: str = "未检查"
    controller_active: bool = False


class DashboardNode(Node):
    def __init__(self, state: DashboardState) -> None:
        super().__init__("spacemouse_franka_impedance_dashboard")
        self.state = state

        self.declare_parameter("action_topic", "/spacemouse_franka_teleop/action")
        self.declare_parameter("deadman_topic", "/spacemouse_franka_teleop/deadman")
        self.declare_parameter("fine_mode_topic", "/spacemouse_franka_teleop/fine_mode")
        self.declare_parameter("target_pose_topic", "/serl_cartesian_impedance_controller/target_pose")
        self.declare_parameter("measured_pose_topic", "/franka_robot_state_broadcaster/current_pose")
        self.declare_parameter(
            "controller_raw_target_pose_topic",
            "/serl_cartesian_impedance_controller/debug/raw_target_pose",
        )
        self.declare_parameter(
            "controller_smoothed_target_pose_topic",
            "/serl_cartesian_impedance_controller/debug/smoothed_target_pose",
        )
        self.declare_parameter(
            "controller_clipped_target_pose_topic",
            "/serl_cartesian_impedance_controller/debug/clipped_target_pose",
        )
        self.declare_parameter(
            "controller_measured_pose_topic",
            "/serl_cartesian_impedance_controller/debug/measured_pose",
        )
        self.declare_parameter(
            "controller_status_topic",
            "/serl_cartesian_impedance_controller/debug/status",
        )
        self.declare_parameter("server_status_topic", "/spacemouse_franka_teleop/server_status")
        self.declare_parameter(
            "external_wrench_topic",
            "/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame",
        )
        self.declare_parameter("controller_manager_service", "/controller_manager/list_controllers")
        self.declare_parameter("controller_name", "serl_cartesian_impedance_controller")
        self.declare_parameter("display_rate_hz", 5.0)
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("project_dir", str(PROJECT_DIR))
        self.declare_parameter("log_dir", str(DEFAULT_LOG_DIR))
        self.declare_parameter("clear_script", "scripts/clear_ros.sh")
        self.declare_parameter("controller_start_script", "scripts/start_serl_cartesian_controller.sh")
        self.declare_parameter("teleop_start_script", "scripts/run_teleop.sh")

        self.controller_name = str(self.get_parameter("controller_name").value)
        self.controller_client = self.create_client(
            ListControllers, str(self.get_parameter("controller_manager_service").value)
        )
        self.pending_controller_future = None

        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("action_topic").value),
            self._action_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("deadman_topic").value),
            self._deadman_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("fine_mode_topic").value),
            self._fine_cb,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_pose_topic").value),
            self._target_pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("measured_pose_topic").value),
            self._measured_pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("controller_raw_target_pose_topic").value),
            self._raw_target_pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("controller_smoothed_target_pose_topic").value),
            self._smoothed_target_pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("controller_clipped_target_pose_topic").value),
            self._clipped_target_pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("controller_measured_pose_topic").value),
            self._controller_measured_pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("controller_status_topic").value),
            self._controller_status_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("server_status_topic").value),
            self._server_status_cb,
            10,
        )
        external_wrench_topic = str(self.get_parameter("external_wrench_topic").value)
        if external_wrench_topic:
            self.create_subscription(
                WrenchStamped,
                external_wrench_topic,
                self._wrench_cb,
                qos_profile_sensor_data,
            )

        self.create_timer(1.0, self._check_controller)

    def _stamp(self) -> float:
        return time.monotonic()

    def _action_cb(self, msg: TwistStamped) -> None:
        with self.state.lock:
            self.state.action = twist_to_array(msg)
            self.state.action_time = self._stamp()

    def _deadman_cb(self, msg: Bool) -> None:
        with self.state.lock:
            self.state.deadman = bool(msg.data)
            self.state.deadman_time = self._stamp()

    def _fine_cb(self, msg: Bool) -> None:
        with self.state.lock:
            self.state.fine_mode = bool(msg.data)
            self.state.fine_mode_time = self._stamp()

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        with self.state.lock:
            self.state.target_pose = pose_to_array(msg)
            self.state.target_pose_time = self._stamp()

    def _measured_pose_cb(self, msg: PoseStamped) -> None:
        with self.state.lock:
            self.state.measured_pose = pose_to_array(msg)
            self.state.measured_pose_time = self._stamp()

    def _raw_target_pose_cb(self, msg: PoseStamped) -> None:
        with self.state.lock:
            self.state.raw_target_pose = pose_to_array(msg)
            self.state.raw_target_pose_time = self._stamp()

    def _smoothed_target_pose_cb(self, msg: PoseStamped) -> None:
        with self.state.lock:
            self.state.smoothed_target_pose = pose_to_array(msg)
            self.state.smoothed_target_pose_time = self._stamp()

    def _clipped_target_pose_cb(self, msg: PoseStamped) -> None:
        with self.state.lock:
            self.state.clipped_target_pose = pose_to_array(msg)
            self.state.clipped_target_pose_time = self._stamp()

    def _controller_measured_pose_cb(self, msg: PoseStamped) -> None:
        with self.state.lock:
            self.state.controller_measured_pose = pose_to_array(msg)
            self.state.controller_measured_pose_time = self._stamp()

    def _controller_status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.controller_status_raw = msg.data
            self.state.controller_status = parse_status(msg.data)
            self.state.controller_status_time = self._stamp()

    def _server_status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.server_status_raw = msg.data
            self.state.server_status = parse_status(msg.data)
            self.state.server_status_time = self._stamp()

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        with self.state.lock:
            self.state.external_wrench = wrench_to_array(msg)
            self.state.external_wrench_time = self._stamp()

    def _check_controller(self) -> None:
        if self.pending_controller_future is not None:
            return
        if not self.controller_client.service_is_ready():
            with self.state.lock:
                self.state.controller_state = "controller_manager 不可用"
                self.state.controller_active = False
            return
        self.pending_controller_future = self.controller_client.call_async(ListControllers.Request())
        self.pending_controller_future.add_done_callback(self._list_controllers_done)

    def _list_controllers_done(self, future) -> None:
        state = "未加载"
        active = False
        try:
            result = future.result()
            for controller in result.controller:
                if controller.name == self.controller_name:
                    state = controller.state
                    active = controller.state == "active"
                    break
        except Exception as exc:
            state = f"查询失败: {exc}"
        with self.state.lock:
            self.state.controller_state = state
            self.state.controller_active = active
        self.pending_controller_future = None


class DashboardApp:
    def __init__(self, root: tk.Tk, state: DashboardState, node: DashboardNode) -> None:
        self.root = root
        self.state = state
        self.node = node
        self.stale_timeout_s = float(node.get_parameter("stale_timeout_s").value)
        self.display_rate_hz = float(node.get_parameter("display_rate_hz").value)
        self.project_dir = Path(str(node.get_parameter("project_dir").value)).expanduser()
        self.log_dir = Path(str(node.get_parameter("log_dir").value)).expanduser()
        self.clear_script = str(node.get_parameter("clear_script").value)
        self.controller_start_script = str(node.get_parameter("controller_start_script").value)
        self.teleop_start_script = str(node.get_parameter("teleop_start_script").value)
        self.vars: dict[str, tk.StringVar] = {}
        self.processes: dict[str, subprocess.Popen] = {}
        self.process_lock = threading.Lock()
        self.record_stop_event = threading.Event()
        self.record_thread: threading.Thread | None = None
        self.record_temp_path: Path | None = None
        self.record_start_monotonic = 0.0
        self.record_frame_count = 0
        self.recording = False
        self.log_dir.mkdir(parents=True, exist_ok=True)

        root.title("Franka SpaceMouse Teleop")
        root.geometry("1180x620")
        root.minsize(980, 560)
        root.configure(bg="#f4f6f8")
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._build()
        self._refresh()

    def _build(self) -> None:
        style = ttk.Style()
        style.configure("Title.TLabel", font=("TkDefaultFont", 15, "bold"))
        style.configure("Value.TLabel", font=("TkFixedFont", 12))
        style.configure("Status.TLabel", font=("TkDefaultFont", 10, "bold"))
        style.configure("Danger.TButton", foreground="#9a1f14")
        style.configure("Alert.TLabel", foreground="#9b1c1c", font=("TkDefaultFont", 12, "bold"))
        style.configure("Ok.TLabel", foreground="#17633a", font=("TkDefaultFont", 12, "bold"))

        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(1, weight=1)

        title = ttk.Label(main, text="Franka SpaceMouse Teleop", style="Title.TLabel")
        title.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))

        left = ttk.Frame(main)
        left.grid(row=1, column=0, sticky="nsw", padx=(0, 12))
        right = ttk.Frame(main)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        controls = ttk.LabelFrame(left, text="Control")
        controls.pack(fill=tk.X)
        self._button(controls, "Clear", self._clear_ros, "Danger.TButton")
        self._button(
            controls,
            "Controller",
            lambda: self._start_process("controller", self.controller_start_script),
        )
        self._button(controls, "Teleop", lambda: self._start_process("teleop", self.teleop_start_script))

        recording = ttk.LabelFrame(left, text="Logging")
        recording.pack(fill=tk.X, pady=(12, 0))
        self._button(recording, "Start Recording", self._start_recording)
        self._button(recording, "Stop Recording", self._stop_recording)
        self.record_status_var = tk.StringVar(value="not recording")
        ttk.Label(recording, textvariable=self.record_status_var, wraplength=260).pack(
            anchor="w", padx=8, pady=(2, 8)
        )

        status = ttk.LabelFrame(left, text="Process Status")
        status.pack(fill=tk.X, pady=(12, 0))
        self.process_status_vars: dict[str, tk.StringVar] = {}
        for name in ("controller", "teleop"):
            var = tk.StringVar(value="stopped")
            self.process_status_vars[name] = var
            ttk.Label(status, text=name.capitalize()).pack(anchor="w", padx=8, pady=(6, 0))
            ttk.Label(status, textvariable=var).pack(anchor="w", padx=8, pady=(0, 4))

        ros = ttk.LabelFrame(left, text="ROS")
        ros.pack(fill=tk.X, pady=(12, 0))
        self.ros_status_var = tk.StringVar(value="starting")
        ttk.Label(ros, text="Dashboard node").pack(anchor="w", padx=8, pady=(6, 0))
        ttk.Label(ros, textvariable=self.ros_status_var).pack(anchor="w", padx=8)
        ttk.Label(ros, text="Controller").pack(anchor="w", padx=8, pady=(8, 0))
        self.controller_ros_var = tk.StringVar(value="waiting")
        ttk.Label(ros, textvariable=self.controller_ros_var, wraplength=260).pack(
            anchor="w", padx=8, pady=(0, 8)
        )

        safety = ttk.LabelFrame(left, text="Safety Gates")
        safety.pack(fill=tk.X, pady=(12, 0))
        self.safety_status_var = tk.StringVar(value="waiting")
        ttk.Label(safety, textvariable=self.safety_status_var, wraplength=300, justify=tk.LEFT).pack(
            anchor="w", padx=8, pady=8
        )

        values = ttk.Frame(right)
        values.grid(row=0, column=0, sticky="ew")
        for column in range(4):
            values.columnconfigure(column, weight=1)

        self.spacemouse_var = self._value_panel(values, "SpaceMouse Command", 0)
        self.measured_pose_var = self._value_panel(values, "Franka Absolute Pose", 1)
        self.target_pose_var = self._value_panel(values, "Target Pose", 2)
        self.controller_clipped_var = self._value_panel(values, "Controller Clipped Target", 3)

        monitor = ttk.Frame(right)
        monitor.grid(row=1, column=0, sticky="nsew", pady=(12, 0))
        monitor.columnconfigure(0, weight=1)

        alert_box = ttk.LabelFrame(monitor, text="Discontinuity Alert")
        alert_box.pack(fill=tk.X)
        self.alert_var = tk.StringVar(value="OK")
        self.alert_label = ttk.Label(alert_box, textvariable=self.alert_var, style="Ok.TLabel", wraplength=760)
        self.alert_label.pack(anchor="w", padx=10, pady=10)

        error_box = ttk.LabelFrame(monitor, text="Pose Error Monitor")
        error_box.pack(fill=tk.X, pady=(12, 0))
        self.pose_error_var = tk.StringVar(value="waiting")
        ttk.Label(error_box, textvariable=self.pose_error_var, wraplength=820, justify=tk.LEFT).pack(
            anchor="w", padx=10, pady=10
        )

        controller_box = ttk.LabelFrame(monitor, text="Controller Status")
        controller_box.pack(fill=tk.X, pady=(12, 0))
        self.controller_status_var = tk.StringVar(value="waiting")
        ttk.Label(controller_box, textvariable=self.controller_status_var, wraplength=820, justify=tk.LEFT).pack(
            anchor="w", padx=10, pady=10
        )

        server_box = ttk.LabelFrame(monitor, text="Server Status")
        server_box.pack(fill=tk.BOTH, expand=True, pady=(12, 0))
        self.server_status_var = tk.StringVar(value="waiting")
        ttk.Label(server_box, textvariable=self.server_status_var, wraplength=820, justify=tk.LEFT).pack(
            anchor="nw", padx=10, pady=10
        )

        self.log_var = tk.StringVar(value=f"logs: {self.log_dir}")
        ttk.Label(right, textvariable=self.log_var).grid(row=2, column=0, sticky="w", pady=(8, 0))

    def _button(self, parent: ttk.Frame, text: str, command, style: str | None = None) -> None:
        button = ttk.Button(parent, text=text, command=command, style=style or "TButton")
        button.pack(fill=tk.X, padx=8, pady=(8, 0))

    def _value_panel(self, parent: ttk.Frame, title: str, column: int) -> tk.StringVar:
        frame = ttk.LabelFrame(parent, text=title)
        frame.grid(row=0, column=column, sticky="nsew", padx=(0 if column == 0 else 8, 0))
        var = tk.StringVar(value="waiting")
        ttk.Label(frame, textvariable=var, style="Value.TLabel", justify=tk.LEFT).pack(
            anchor="w", padx=10, pady=10
        )
        return var

    def _add_status_item(self, parent: ttk.Frame, key: str, title: str) -> None:
        frame = ttk.Frame(parent, padding=(0, 0, 18, 0))
        frame.pack(side="left", fill="x", expand=True)
        ttk.Label(frame, text=title).pack(anchor="w")
        var = tk.StringVar(value="等待")
        ttk.Label(frame, textvariable=var, style="Status.TLabel").pack(anchor="w")
        self.vars[key] = var

    def _add_panel(self, parent: ttk.Frame, key: str, title: str, row: int, column: int) -> None:
        frame = ttk.LabelFrame(parent, text=title, style="Section.TLabelframe", padding=8)
        frame.grid(row=row, column=column, sticky="nsew", padx=5, pady=5)
        var = tk.StringVar(value="等待数据")
        label = ttk.Label(frame, textvariable=var, style="Value.TLabel", wraplength=340)
        label.pack(anchor="nw", fill="both", expand=True)
        self.vars[key] = var

    def _snapshot(self) -> DashboardState:
        with self.state.lock:
            return DashboardState(
                action=None if self.state.action is None else self.state.action.copy(),
                action_time=self.state.action_time,
                deadman=self.state.deadman,
                deadman_time=self.state.deadman_time,
                fine_mode=self.state.fine_mode,
                fine_mode_time=self.state.fine_mode_time,
                target_pose=None if self.state.target_pose is None else self.state.target_pose.copy(),
                target_pose_time=self.state.target_pose_time,
                measured_pose=None
                if self.state.measured_pose is None
                else self.state.measured_pose.copy(),
                measured_pose_time=self.state.measured_pose_time,
                raw_target_pose=None
                if self.state.raw_target_pose is None
                else self.state.raw_target_pose.copy(),
                raw_target_pose_time=self.state.raw_target_pose_time,
                smoothed_target_pose=None
                if self.state.smoothed_target_pose is None
                else self.state.smoothed_target_pose.copy(),
                smoothed_target_pose_time=self.state.smoothed_target_pose_time,
                clipped_target_pose=None
                if self.state.clipped_target_pose is None
                else self.state.clipped_target_pose.copy(),
                clipped_target_pose_time=self.state.clipped_target_pose_time,
                controller_measured_pose=None
                if self.state.controller_measured_pose is None
                else self.state.controller_measured_pose.copy(),
                controller_measured_pose_time=self.state.controller_measured_pose_time,
                external_wrench=None
                if self.state.external_wrench is None
                else self.state.external_wrench.copy(),
                external_wrench_time=self.state.external_wrench_time,
                server_status_raw=self.state.server_status_raw,
                server_status=dict(self.state.server_status),
                server_status_time=self.state.server_status_time,
                controller_status_raw=self.state.controller_status_raw,
                controller_status=dict(self.state.controller_status),
                controller_status_time=self.state.controller_status_time,
                controller_state=self.state.controller_state,
                controller_active=self.state.controller_active,
            )

    def _refresh(self) -> None:
        now = time.monotonic()
        s = self._snapshot()

        controller_label = "active" if s.controller_active else s.controller_state
        self.ros_status_var.set("running")
        self.controller_ros_var.set(f"{self.node.controller_name}: {controller_label}")
        self._refresh_process_status()

        self.spacemouse_var.set(
            f"{fmt_vec(s.action, ('x', 'y', 'z', 'rx', 'ry', 'rz'))}\n"
            f"deadman: {s.deadman}\n"
            f"fine_mode: {s.fine_mode}\n"
            f"action_age: {fmt_age(s.action_time, now, self.stale_timeout_s)}"
        )
        self.measured_pose_var.set(
            f"{fmt_pose(s.measured_pose)}\n"
            f"age: {fmt_age(s.measured_pose_time, now, self.stale_timeout_s)}"
        )
        self.target_pose_var.set(
            f"{fmt_pose(s.target_pose)}\n"
            f"age: {fmt_age(s.target_pose_time, now, self.stale_timeout_s)}"
        )
        self.controller_clipped_var.set(
            f"{fmt_pose(s.clipped_target_pose)}\n"
            f"age: {fmt_age(s.clipped_target_pose_time, now, self.stale_timeout_s)}"
        )

        target_measured = pose_error(s.target_pose, s.measured_pose)
        raw_smoothed = pose_error(s.raw_target_pose, s.smoothed_target_pose)
        smoothed_clipped = pose_error(s.smoothed_target_pose, s.clipped_target_pose)
        clipped_measured = pose_error(s.clipped_target_pose, s.controller_measured_pose)
        alert_text, alert_ok = self._alert_text(s, target_measured, smoothed_clipped)
        self.alert_var.set(alert_text)
        self.alert_label.configure(style="Ok.TLabel" if alert_ok else "Alert.TLabel")
        self.pose_error_var.set(
            f"target - measured: {fmt_mm(target_measured)} mm\n"
            f"raw - smoothed: {fmt_mm(raw_smoothed)} mm\n"
            f"smoothed - clipped: {fmt_mm(smoothed_clipped)} mm\n"
            f"clipped - controller measured: {fmt_mm(clipped_measured)} mm\n"
            f"raw age: {fmt_age(s.raw_target_pose_time, now, self.stale_timeout_s)}  "
            f"smoothed age: {fmt_age(s.smoothed_target_pose_time, now, self.stale_timeout_s)}"
        )
        self.safety_status_var.set(self._safety_text(s, now))
        self.controller_status_var.set(
            self._status_block(
                s.controller_status,
                s.controller_status_raw,
                (
                    "controller_active",
                    "target_received",
                    "target_update_count",
                    "target_age_s",
                    "reference_was_clipped",
                    "position_error_before_clip",
                    "position_error_after_clip",
                    "orientation_error_before_clip",
                    "orientation_error_after_clip",
                    "tau_norm",
                    "tau_rate_limited",
                    "update_period",
                ),
            )
        )

        wrench = s.external_wrench
        if wrench is None:
            wrench_line = "wrench: 等待数据"
        else:
            force_norm = float(np.linalg.norm(wrench[:3]))
            torque_norm = float(np.linalg.norm(wrench[3:6]))
            wrench_line = f"force_norm={fmt_float(force_norm, 5)} torque_norm={fmt_float(torque_norm, 5)}"
        self.server_status_var.set(
            self._status_block(
                s.server_status,
                s.server_status_raw,
                (
                    "state",
                    "reason",
                    "controller_state",
                    "publish_enabled",
                    "target_publish_count",
                    "target_measured_error_norm",
                    "external_force_norm",
                    "external_torque_norm",
                ),
            )
            + f"\n{wrench_line}"
        )
        if self.recording:
            self.record_status_var.set(
                f"recording {self.record_frame_count} frames -> {self.record_temp_path}"
            )

        period_ms = int(1000.0 / max(0.2, self.display_rate_hz))
        self.root.after(period_ms, self._refresh)

    def _status_block(
        self,
        status: dict[str, str],
        raw: str,
        preferred_keys: tuple[str, ...],
    ) -> str:
        if not raw:
            return "等待 status topic"
        distance_keys = {
            "position_error_before_clip",
            "position_error_after_clip",
        }
        lines = []
        for key in preferred_keys:
            value = status.get(key)
            if value is None:
                lines.append(f"{key}: 等待")
                continue
            if key in distance_keys:
                try:
                    lines.append(f"{key}: {float(value) * 1000.0:.3f} mm")
                except ValueError:
                    lines.append(f"{key}: {value}")
            else:
                lines.append(f"{key}: {value}")
        return "\n".join(lines)

    def _resolve_script(self, script_name: str) -> Path:
        path = Path(script_name).expanduser()
        if path.is_absolute():
            return path
        return self.project_dir / path

    def _start_process(self, name: str, script_name: str) -> None:
        with self.process_lock:
            proc = self.processes.get(name)
            if proc is not None and proc.poll() is None:
                self.log_var.set(f"{name} 已在运行，pid={proc.pid}")
                return

        script = self._resolve_script(script_name)
        log_path = self.log_dir / f"spacemouse_franka_{name}.log"
        if not script.exists():
            self.log_var.set(f"{name} 启动失败：找不到 {script}")
            return

        env = os.environ.copy()
        env.setdefault("SPACEMOUSE_FRANKA_TELEOP_DIR", str(self.project_dir))
        log_file = None
        try:
            log_file = log_path.open("ab", buffering=0)
            proc = subprocess.Popen(
                [str(script)],
                cwd=str(self.project_dir),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
        except Exception as exc:
            self.log_var.set(f"{name} 启动失败：{exc}")
            return
        finally:
            if log_file is not None:
                log_file.close()

        with self.process_lock:
            self.processes[name] = proc
        self.log_var.set(f"已启动 {name}，pid={proc.pid}，log={log_path}")

    def _clear_ros(self) -> None:
        self.log_var.set("正在清理 ROS / teleop 进程...")
        threading.Thread(target=self._clear_ros_worker, daemon=True).start()

    def _clear_ros_worker(self) -> None:
        script = self._resolve_script(self.clear_script)
        log_path = self.log_dir / "spacemouse_franka_clear.log"

        with self.process_lock:
            processes = list(self.processes.items())
            self.processes.clear()

        for _name, proc in processes:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except Exception:
                    pass

        with log_path.open("ab", buffering=0) as log_file:
            if script.exists():
                subprocess.run(
                    [str(script)],
                    cwd=str(self.project_dir),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    env=os.environ.copy(),
                    check=False,
                )
                message = f"清理完成，log={log_path}"
            else:
                log_file.write(f"missing clear script: {script}\n".encode("utf-8"))
                message = f"清理失败：找不到 {script}"
        self.root.after(0, lambda: self.log_var.set(message))

    def _process_summary(self) -> str:
        parts = []
        with self.process_lock:
            for name in ("controller", "teleop"):
                proc = self.processes.get(name)
                if proc is None:
                    parts.append(f"{name}=未启动")
                    continue
                code = proc.poll()
                if code is None:
                    parts.append(f"{name}=运行中(pid {proc.pid})")
                else:
                    parts.append(f"{name}=退出({code})")
        return "进程: " + "  ".join(parts)

    def _refresh_process_status(self) -> None:
        with self.process_lock:
            snapshot = dict(self.processes)
        for name, var in self.process_status_vars.items():
            proc = snapshot.get(name)
            if proc is None:
                var.set("stopped")
                continue
            code = proc.poll()
            if code is None:
                var.set(f"running, pid={proc.pid}")
            else:
                var.set(f"exited, code={code}")

    def _safety_text(self, s: DashboardState, now: float) -> str:
        return "\n".join(
            [
                f"controller_active: {s.controller_active}",
                f"controller_state: {s.controller_state}",
                f"server_state: {s.server_status.get('state', 'waiting')}",
                f"server_reason: {s.server_status.get('reason', 'waiting')}",
                f"measured_pose: {fmt_age(s.measured_pose_time, now, self.stale_timeout_s)}",
                f"target_pose: {fmt_age(s.target_pose_time, now, self.stale_timeout_s)}",
                f"controller_status: {fmt_age(s.controller_status_time, now, self.stale_timeout_s)}",
                f"reference_clipped: {s.controller_status.get('reference_was_clipped', s.controller_status.get('reference_clipped', 'waiting'))}",
            ]
        )

    def _alert_text(
        self, s: DashboardState, target_measured: float, smoothed_clipped: float
    ) -> tuple[str, bool]:
        if not s.controller_active:
            return (f"等待 controller active: {s.controller_state}", False)
        if s.measured_pose_time <= 0.0:
            return ("等待 Franka measured pose", False)
        if s.target_pose_time <= 0.0:
            return ("等待 target pose", False)
        if math.isfinite(target_measured) and target_measured > 0.010:
            return (f"target-measured error 偏大: {target_measured * 1000.0:.3f} mm", False)
        if math.isfinite(smoothed_clipped) and smoothed_clipped > 0.003:
            return (f"smoothed-clipped error 偏大: {smoothed_clipped * 1000.0:.3f} mm", False)
        clipped_value = s.controller_status.get(
            "reference_was_clipped", s.controller_status.get("reference_clipped", "")
        )
        if clipped_value.lower() in ("true", "1"):
            return ("controller reference clipping active", False)
        return ("OK", True)

    def _start_recording(self) -> None:
        if self.record_thread is not None and self.record_thread.is_alive():
            self.log_var.set("recording already running")
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.record_temp_path = self.log_dir / f"spacemouse_franka_recording_{timestamp}.tmp.csv"
        self.record_stop_event.clear()
        self.record_start_monotonic = time.monotonic()
        self.record_frame_count = 0
        self.recording = True
        self.record_thread = threading.Thread(target=self._record_worker, daemon=True)
        self.record_thread.start()
        self.record_status_var.set(f"recording to temp: {self.record_temp_path}")
        self.log_var.set("recording started")

    def _stop_recording(self) -> None:
        if self.record_thread is None or not self.record_thread.is_alive():
            self.recording = False
            self.record_status_var.set("not recording")
            self.log_var.set("recording is not running")
            return
        self.record_stop_event.set()
        self.record_thread.join(timeout=3.0)
        self.recording = False
        temp_path = self.record_temp_path
        if temp_path is None or not temp_path.exists():
            self.record_status_var.set("recording stopped, no file produced")
            self.log_var.set("recording stopped, no file produced")
            return
        default_name = f"spacemouse_franka_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        save_path = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save SpaceMouse Franka recording",
            initialdir=str(self.project_dir),
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not save_path:
            self.record_status_var.set(f"recording stopped; temp kept: {temp_path}")
            self.log_var.set(f"save canceled, temp kept: {temp_path}")
            return
        destination = Path(save_path)
        try:
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(temp_path, destination)
            temp_path.unlink(missing_ok=True)
        except Exception as exc:
            self.record_status_var.set(f"save failed: {exc}")
            self.log_var.set(f"save failed: {exc}")
            return
        self.record_status_var.set(f"saved {self.record_frame_count} frames: {destination}")
        self.log_var.set(f"recording saved: {destination}")

    def _record_worker(self) -> None:
        temp_path = self.record_temp_path
        if temp_path is None:
            return
        frame = 0
        next_time = time.monotonic()
        with temp_path.open("w", newline="", buffering=1) as file:
            writer = csv.writer(file)
            writer.writerow(RECORD_HEADER)
            while not self.record_stop_event.is_set():
                writer.writerow(self._record_snapshot(frame))
                frame += 1
                self.record_frame_count = frame
                next_time += RECORD_PERIOD_S
                sleep_s = next_time - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_time = time.monotonic()

    def _record_snapshot(self, frame: int) -> list[object]:
        now_mono = time.monotonic()
        now_unix = time.time()
        s = self._snapshot()
        process_state = self._process_record_state()
        target_measured = pose_error(s.target_pose, s.measured_pose)
        raw_smoothed = pose_error(s.raw_target_pose, s.smoothed_target_pose)
        smoothed_clipped = pose_error(s.smoothed_target_pose, s.clipped_target_pose)
        clipped_measured = pose_error(s.clipped_target_pose, s.controller_measured_pose)
        if s.target_pose is None or s.measured_pose is None:
            target_minus_measured = [math.nan, math.nan, math.nan]
        else:
            target_minus_measured = [float(v) for v in (s.target_pose[:3] - s.measured_pose[:3])]
        alert, _ok = self._alert_text(s, target_measured, smoothed_clipped)
        return [
            frame,
            f"{now_unix:.9f}",
            f"{now_mono - self.record_start_monotonic:.9f}",
            process_state["controller_running"],
            process_state["controller_exit_code"],
            process_state["teleop_running"],
            process_state["teleop_exit_code"],
            "DEADMAN" if s.deadman else "IDLE",
            int(s.deadman),
            int(s.fine_mode),
            *_array_values_for_csv(s.action, 6),
            _age_for_csv(now_mono, s.action_time),
            *_pose_values_for_csv(s.measured_pose),
            _age_for_csv(now_mono, s.measured_pose_time),
            *_pose_values_for_csv(s.target_pose),
            _age_for_csv(now_mono, s.target_pose_time),
            *_pose_values_for_csv(s.raw_target_pose),
            _age_for_csv(now_mono, s.raw_target_pose_time),
            *_pose_values_for_csv(s.smoothed_target_pose),
            _age_for_csv(now_mono, s.smoothed_target_pose_time),
            *_pose_values_for_csv(s.clipped_target_pose),
            _age_for_csv(now_mono, s.clipped_target_pose_time),
            *_pose_values_for_csv(s.controller_measured_pose),
            _age_for_csv(now_mono, s.controller_measured_pose_time),
            *[f"{value:.12f}" if math.isfinite(value) else "nan" for value in target_minus_measured],
            _float_for_csv(target_measured),
            _float_for_csv(raw_smoothed),
            _float_for_csv(smoothed_clipped),
            _float_for_csv(clipped_measured),
            int(s.controller_active),
            s.controller_state,
            s.server_status.get("state", ""),
            s.server_status.get("reason", ""),
            s.server_status.get("target_publish_count", ""),
            s.controller_status.get("reference_clipped", ""),
            s.controller_status.get("target_distance_clamped", ""),
            s.controller_status.get("desired_speed_limited", ""),
            s.controller_status.get("desired_acceleration_limited", ""),
            s.controller_status.get("jerk_limited", ""),
            s.controller_status.get("step_limited", ""),
            s.controller_status.get("commanded_torque_norm", ""),
            *[s.controller_status.get(field, "") for field in CONTROLLER_DEBUG_RECORD_FIELDS],
            s.server_status.get("external_force_norm", ""),
            s.server_status.get("external_torque_norm", ""),
            s.server_status_raw,
            s.controller_status_raw,
            alert,
        ]

    def _process_record_state(self) -> dict[str, int | str]:
        with self.process_lock:
            snapshot = dict(self.processes)
        result: dict[str, int | str] = {}
        for name in ("controller", "teleop"):
            proc = snapshot.get(name)
            if proc is None:
                running = 0
                exit_code: int | str = ""
            else:
                code = proc.poll()
                running = int(code is None)
                exit_code = "" if code is None else int(code)
            result[f"{name}_running"] = running
            result[f"{name}_exit_code"] = exit_code
        return result

    def _on_close(self) -> None:
        if self.record_thread is not None and self.record_thread.is_alive():
            self.record_stop_event.set()
            self.record_thread.join(timeout=1.0)
        self.root.destroy()


def main() -> None:
    rclpy.init()
    state = DashboardState()
    node = DashboardNode(state)

    def spin_node() -> None:
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = DashboardApp(root, state, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        del app


if __name__ == "__main__":
    main()
