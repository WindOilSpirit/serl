#!/usr/bin/env python3
"""Tk dashboard for starting and monitoring SpaceMouse Franka teleop."""

from __future__ import annotations

import os
import csv
import signal
import shutil
import subprocess
import threading
import time
from datetime import datetime
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np
import tkinter as tk
from tkinter import filedialog, ttk

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


PROJECT_DIR = Path(
    os.environ.get(
        "SPACEMOUSE_FRANKA_TELEOP_DIR",
        "/home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test",
    )
)
LOG_DIR = Path(os.environ.get("SPACEMOUSE_FRANKA_UI_LOG_DIR", "/tmp"))
RECORD_RATE_HZ = 1000.0
RECORD_PERIOD_S = 1.0 / RECORD_RATE_HZ
POSE_FIELD_NAMES = ("x", "y", "z", "qx", "qy", "qz", "qw")
RECORD_HEADER = [
    "frame",
    "time_unix_s",
    "time_since_start_s",
    "machine_state",
    "bringup_running",
    "bringup_exit_code",
    "controller_running",
    "controller_exit_code",
    "teleop_running",
    "teleop_exit_code",
    "teleop_state",
    "fine_mode",
    "spacemouse_linear_x",
    "spacemouse_linear_y",
    "spacemouse_linear_z",
    "spacemouse_angular_x",
    "spacemouse_angular_y",
    "spacemouse_angular_z",
    "action_age_s",
    *[f"current_{name}" for name in POSE_FIELD_NAMES],
    "current_pose_age_s",
    *[f"target_{name}" for name in POSE_FIELD_NAMES],
    "target_pose_age_s",
    "target_pose_fresh",
    *[f"controller_raw_received_target_{name}" for name in POSE_FIELD_NAMES],
    "controller_raw_received_target_age_s",
    *[f"controller_accepted_target_{name}" for name in POSE_FIELD_NAMES],
    "controller_accepted_target_age_s",
    *[f"controller_received_target_{name}" for name in POSE_FIELD_NAMES],
    "controller_received_target_age_s",
    *[f"controller_rt_target_{name}" for name in POSE_FIELD_NAMES],
    "controller_rt_target_age_s",
    *[f"controller_internal_command_{name}" for name in POSE_FIELD_NAMES],
    "controller_internal_command_age_s",
    "target_minus_current_x",
    "target_minus_current_y",
    "target_minus_current_z",
    "target_minus_controller_internal_command_x",
    "target_minus_controller_internal_command_y",
    "target_minus_controller_internal_command_z",
    "controller_internal_command_minus_current_x",
    "controller_internal_command_minus_current_y",
    "controller_internal_command_minus_current_z",
    "controller_process_running",
    "ros2_controller_active",
    "command_topic_connected",
    "robot_state_fresh",
    "target_seed_source",
    "first_target_source",
    "first_target_to_internal_command_norm",
    "first_target_to_measured_norm",
    "target_initialized_from_controller_command_pose",
    "control_target_valid_for_motion",
    "bottom_controller_targets_enabled",
    "controller_enable_targets_success",
    "publish_block_reason",
    "target_write_reason",
    "controller_accept_targets",
    "controller_target_accepted_count",
    "controller_target_rejected_count",
    "controller_last_target_reject_reason",
    "controller_target_to_command_error",
    "controller_target_to_measured_error",
    "controller_command_to_measured_error",
    "activation_command_to_measured_norm",
    "seeded_command_to_measured_norm",
    "command_seed_source",
    "controller_update_period_s",
    "controller_update_overrun_count",
    "controller_target_stream_primed",
    "controller_has_target",
    "controller_rt_has_target",
    "server_status",
    "alert",
]


@dataclass
class DashboardState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    spacemouse: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))
    teleop_state: str = "unknown"
    deadman_pressed: bool = False
    fine_mode: bool = False
    current_pose: np.ndarray | None = None
    target_pose: np.ndarray | None = None
    controller_raw_received_target_pose: np.ndarray | None = None
    controller_received_target_pose: np.ndarray | None = None
    controller_rt_target_pose: np.ndarray | None = None
    controller_internal_command_pose: np.ndarray | None = None
    last_action_time: float = 0.0
    last_current_pose_time: float = 0.0
    last_target_pose_time: float = 0.0
    last_controller_raw_received_target_time: float = 0.0
    last_controller_received_target_time: float = 0.0
    last_controller_rt_target_time: float = 0.0
    last_controller_internal_command_time: float = 0.0
    ros_status: str = "starting"
    server_status: str = "waiting"
    controller_target_status: str = "waiting"
    alert: str = "OK"
    reset_status: str = "not requested"


class DashboardNode:
    def __init__(self, state: DashboardState) -> None:
        self.state = state
        self.node = rclpy.create_node("spacemouse_franka_teleop_dashboard")
        self.subscriptions = [
            self.node.create_subscription(
                TwistStamped,
                "/spacemouse_franka_teleop/action",
                self._action_cb,
                10,
            ),
            self.node.create_subscription(
                Bool,
                "/spacemouse_franka_teleop/deadman",
                self._deadman_cb,
                10,
            ),
            self.node.create_subscription(
                Bool,
                "/spacemouse_franka_teleop/fine_mode",
                self._fine_cb,
                10,
            ),
            self.node.create_subscription(
                PoseStamped,
                "/franka_robot_state_broadcaster/current_pose",
                self._current_pose_cb,
                10,
            ),
            self.node.create_subscription(
                PoseStamped,
                "/serl_safe_cartesian_pose_controller/target_pose",
                self._target_pose_cb,
                10,
            ),
            self.node.create_subscription(
                PoseStamped,
                "/serl_safe_cartesian_pose_controller/debug/received_target_pose",
                self._controller_raw_received_target_cb,
                10,
            ),
            self.node.create_subscription(
                PoseStamped,
                "/serl_safe_cartesian_pose_controller/debug/accepted_target_pose",
                self._controller_received_target_cb,
                10,
            ),
            self.node.create_subscription(
                PoseStamped,
                "/serl_safe_cartesian_pose_controller/debug/rt_target_pose",
                self._controller_rt_target_cb,
                10,
            ),
            self.node.create_subscription(
                PoseStamped,
                "/serl_safe_cartesian_pose_controller/debug/internal_command_pose",
                self._controller_internal_command_cb,
                10,
            ),
            self.node.create_subscription(
                String,
                "/serl_safe_cartesian_pose_controller/debug/target_status",
                self._controller_target_status_cb,
                10,
            ),
            self.node.create_subscription(
                String,
                "/spacemouse_franka_teleop/server_status",
                self._server_status_cb,
                10,
            ),
        ]
        self.reset_client = self.node.create_client(
            Trigger,
            "/spacemouse_franka_pose_action_server/reset_to_current_pose",
        )
        with self.state.lock:
            self.state.ros_status = "running"

    def destroy(self) -> None:
        self.node.destroy_node()

    def _action_cb(self, msg: TwistStamped) -> None:
        now = time.monotonic()
        values = np.array(
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
        with self.state.lock:
            self.state.spacemouse = values
            self.state.last_action_time = now

    def _deadman_cb(self, msg: Bool) -> None:
        with self.state.lock:
            self.state.deadman_pressed = bool(msg.data)
            self.state.teleop_state = "DEADMAN" if msg.data else "IDLE"

    def _fine_cb(self, msg: Bool) -> None:
        with self.state.lock:
            self.state.fine_mode = bool(msg.data)

    def _current_pose_cb(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        with self.state.lock:
            self.state.current_pose = pose_to_array(msg)
            self.state.last_current_pose_time = now

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        with self.state.lock:
            self.state.target_pose = pose_to_array(msg)
            self.state.last_target_pose_time = now

    def _controller_raw_received_target_cb(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        with self.state.lock:
            self.state.controller_raw_received_target_pose = pose_to_array(msg)
            self.state.last_controller_raw_received_target_time = now

    def _controller_received_target_cb(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        with self.state.lock:
            self.state.controller_received_target_pose = pose_to_array(msg)
            self.state.last_controller_received_target_time = now

    def _controller_rt_target_cb(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        with self.state.lock:
            self.state.controller_rt_target_pose = pose_to_array(msg)
            self.state.last_controller_rt_target_time = now

    def _controller_internal_command_cb(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        with self.state.lock:
            self.state.controller_internal_command_pose = pose_to_array(msg)
            self.state.last_controller_internal_command_time = now

    def _server_status_cb(self, msg: String) -> None:
        text = msg.data
        lowered = text.lower()
        alert = "OK"
        if "discontinuity" in lowered or "tracking guard" in lowered:
            alert = text
        with self.state.lock:
            self.state.server_status = text
            self.state.alert = alert

    def _controller_target_status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.controller_target_status = msg.data

    def request_reset(self) -> str:
        if not self.reset_client.service_is_ready():
            with self.state.lock:
                self.state.reset_status = "reset service unavailable"
            return "reset service unavailable"
        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(self._reset_done_cb)
        with self.state.lock:
            self.state.reset_status = "reset requested"
        return "reset requested"

    def _reset_done_cb(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            status = f"reset failed: {exc}"
        else:
            status = result.message if result.success else f"reset rejected: {result.message}"
        with self.state.lock:
            self.state.reset_status = status
            if "reset failed" not in status and "rejected" not in status:
                self.state.alert = "OK"


class TeleopDashboardApp:
    def __init__(self) -> None:
        self.state = DashboardState()
        self.root = tk.Tk()
        self.root.title("Franka SpaceMouse Teleop")
        self.root.geometry("1180x620")
        self.root.minsize(980, 560)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.processes: dict[str, subprocess.Popen] = {}
        self.ros_thread: threading.Thread | None = None
        self.ros_executor: SingleThreadedExecutor | None = None
        self.ros_node: DashboardNode | None = None
        self.status_vars: dict[str, tk.StringVar] = {}
        self.process_lock = threading.Lock()
        self.record_stop_event = threading.Event()
        self.record_thread: threading.Thread | None = None
        self.record_temp_path: Path | None = None
        self.record_start_monotonic = 0.0
        self.record_frame_count = 0
        self.recording = False

        self._build_ui()
        self._start_ros_thread()
        self._schedule_refresh()

    def run(self) -> None:
        self.root.mainloop()

    def _build_ui(self) -> None:
        style = ttk.Style()
        style.configure("Title.TLabel", font=("TkDefaultFont", 15, "bold"))
        style.configure("Value.TLabel", font=("TkFixedFont", 12))
        style.configure("Danger.TButton", foreground="#8a1f11")
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
        self._button(controls, "Bringup", lambda: self._start_process("bringup", "start_bringup.sh"))
        self._button(
            controls,
            "Controller",
            lambda: self._start_process("controller", "start_serl_cartesian_controller.sh"),
        )
        self._button(controls, "Teleop", lambda: self._start_process("teleop", "run_teleop.sh"))
        self._button(controls, "Reset Target", self._reset_target_to_current)

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
        for name in ("bringup", "controller", "teleop"):
            var = tk.StringVar(value="stopped")
            self.status_vars[name] = var
            ttk.Label(status, text=name.capitalize()).pack(anchor="w", padx=8, pady=(6, 0))
            ttk.Label(status, textvariable=var).pack(anchor="w", padx=8, pady=(0, 4))

        ros = ttk.LabelFrame(left, text="ROS")
        ros.pack(fill=tk.X, pady=(12, 0))
        self.ros_status_var = tk.StringVar(value="starting")
        self.reset_status_var = tk.StringVar(value="not requested")
        ttk.Label(ros, text="Dashboard node").pack(anchor="w", padx=8, pady=(6, 0))
        ttk.Label(ros, textvariable=self.ros_status_var).pack(anchor="w", padx=8)
        ttk.Label(ros, text="Reset").pack(anchor="w", padx=8, pady=(8, 0))
        ttk.Label(ros, textvariable=self.reset_status_var, wraplength=260).pack(
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
        values.columnconfigure(0, weight=1)
        values.columnconfigure(1, weight=1)
        values.columnconfigure(2, weight=1)
        values.columnconfigure(3, weight=1)

        self.spacemouse_var = self._value_panel(values, "SpaceMouse Command", 0)
        self.current_pose_var = self._value_panel(values, "Franka Absolute Pose", 1)
        self.target_pose_var = self._value_panel(values, "Target Pose", 2)
        self.controller_internal_command_var = self._value_panel(
            values, "Controller Internal Command", 3
        )

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

        server_box = ttk.LabelFrame(monitor, text="Server Status")
        server_box.pack(fill=tk.BOTH, expand=True, pady=(12, 0))
        self.server_status_var = tk.StringVar(value="waiting")
        ttk.Label(server_box, textvariable=self.server_status_var, wraplength=820, justify=tk.LEFT).pack(
            anchor="nw", padx=10, pady=10
        )

        self.log_var = tk.StringVar(value=f"logs: {LOG_DIR}")
        ttk.Label(right, textvariable=self.log_var).grid(row=2, column=0, sticky="w", pady=(8, 0))

    def _button(
        self,
        parent: ttk.Frame,
        text: str,
        command: Callable[[], None],
        style: str | None = None,
    ) -> None:
        button = ttk.Button(parent, text=text, command=command, style=style or "TButton")
        button.pack(fill=tk.X, padx=8, pady=5)

    def _value_panel(self, parent: ttk.Frame, title: str, column: int) -> tk.StringVar:
        frame = ttk.LabelFrame(parent, text=title)
        frame.grid(row=0, column=column, sticky="ew", padx=(0, 8 if column < 2 else 0))
        var = tk.StringVar(value="waiting")
        ttk.Label(frame, textvariable=var, style="Value.TLabel", justify=tk.LEFT).pack(
            anchor="w", padx=10, pady=10
        )
        return var

    def _start_ros_thread(self) -> None:
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()

    def _ros_spin(self) -> None:
        try:
            try:
                from rclpy.signals import SignalHandlerOptions

                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            except TypeError:
                rclpy.init()
            self.ros_node = DashboardNode(self.state)
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node.node)
            self.ros_executor.spin()
        except Exception as exc:
            with self.state.lock:
                self.state.ros_status = f"error: {exc}"
        finally:
            if self.ros_node is not None:
                self.ros_node.destroy()
            if rclpy.ok():
                rclpy.shutdown()

    def _reset_target_to_current(self) -> None:
        if self.ros_node is None:
            self.log_var.set("reset unavailable: dashboard ROS node is not ready")
            return
        status = self.ros_node.request_reset()
        self.log_var.set(status)

    def _start_recording(self) -> None:
        if self.record_thread is not None and self.record_thread.is_alive():
            self.log_var.set("recording already running")
            return
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.record_temp_path = LOG_DIR / f"spacemouse_franka_recording_{timestamp}.tmp.csv"
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
            initialdir=str(PROJECT_DIR),
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
        with self.state.lock:
            spacemouse = self.state.spacemouse.copy()
            teleop_state = self.state.teleop_state
            fine_mode = self.state.fine_mode
            current_pose = None if self.state.current_pose is None else self.state.current_pose.copy()
            target_pose = None if self.state.target_pose is None else self.state.target_pose.copy()
            controller_raw_received_target_pose = (
                None
                if self.state.controller_raw_received_target_pose is None
                else self.state.controller_raw_received_target_pose.copy()
            )
            controller_received_target_pose = (
                None
                if self.state.controller_received_target_pose is None
                else self.state.controller_received_target_pose.copy()
            )
            controller_rt_target_pose = (
                None
                if self.state.controller_rt_target_pose is None
                else self.state.controller_rt_target_pose.copy()
            )
            controller_internal_command_pose = (
                None
                if self.state.controller_internal_command_pose is None
                else self.state.controller_internal_command_pose.copy()
            )
            server_status = self.state.server_status
            controller_target_status = self.state.controller_target_status
            alert = self.state.alert
            last_action_time = self.state.last_action_time
            last_current_pose_time = self.state.last_current_pose_time
            last_target_pose_time = self.state.last_target_pose_time
            last_controller_raw_received_target_time = (
                self.state.last_controller_raw_received_target_time
            )
            last_controller_received_target_time = self.state.last_controller_received_target_time
            last_controller_rt_target_time = self.state.last_controller_rt_target_time
            last_controller_internal_command_time = self.state.last_controller_internal_command_time
        process_state = self._process_record_state(now_mono, last_current_pose_time)
        server_flags = _parse_server_status(server_status)
        server_fields = _status_fields(server_status)
        controller_fields = _status_fields(controller_target_status)
        publish_block_reason = _status_value_span(
            server_status, "publish_block_reason", "target_initialized_from_controller_command_pose"
        )
        target_write = _status_value_span(server_status, "write", "target_seed_source")
        target_pose_age = now_mono - last_target_pose_time if last_target_pose_time > 0.0 else math_nan()
        target_pose_fresh = bool(last_target_pose_time > 0.0 and target_pose_age <= 0.25)
        controller_internal_command_age = (
            now_mono - last_controller_internal_command_time
            if last_controller_internal_command_time > 0.0
            else math_nan()
        )
        controller_internal_command_fresh = bool(
            last_controller_internal_command_time > 0.0
            and controller_internal_command_age <= 0.25
        )
        current_values = _pose_values_for_csv(current_pose)
        target_for_csv = target_pose if target_pose_fresh else None
        command_for_error = (
            controller_internal_command_pose if controller_internal_command_fresh else None
        )
        target_values = _pose_values_for_csv(target_for_csv)
        if current_pose is None or target_for_csv is None:
            target_minus_current = [math_nan(), math_nan(), math_nan()]
        else:
            target_minus_current = [float(v) for v in (target_for_csv[:3] - current_pose[:3])]
        if target_for_csv is None or command_for_error is None:
            target_minus_command = [math_nan(), math_nan(), math_nan()]
        else:
            target_minus_command = [
                float(v) for v in (target_for_csv[:3] - command_for_error[:3])
            ]
        if current_pose is None or command_for_error is None:
            command_minus_current = [math_nan(), math_nan(), math_nan()]
        else:
            command_minus_current = [
                float(v) for v in (command_for_error[:3] - current_pose[:3])
            ]
        return [
            frame,
            f"{now_unix:.9f}",
            f"{now_mono - self.record_start_monotonic:.9f}",
            process_state["machine_state"],
            process_state["bringup_running"],
            process_state["bringup_exit_code"],
            process_state["controller_running"],
            process_state["controller_exit_code"],
            process_state["teleop_running"],
            process_state["teleop_exit_code"],
            teleop_state,
            int(fine_mode),
            *[f"{value:.12f}" for value in spacemouse],
            _age_for_csv(now_mono, last_action_time),
            *current_values,
            _age_for_csv(now_mono, last_current_pose_time),
            *target_values,
            _format_age_value(target_pose_age),
            int(target_pose_fresh),
            *_pose_values_for_csv(controller_raw_received_target_pose),
            _age_for_csv(now_mono, last_controller_raw_received_target_time),
            *_pose_values_for_csv(controller_received_target_pose),
            _age_for_csv(now_mono, last_controller_received_target_time),
            *_pose_values_for_csv(controller_received_target_pose),
            _age_for_csv(now_mono, last_controller_received_target_time),
            *_pose_values_for_csv(controller_rt_target_pose),
            _age_for_csv(now_mono, last_controller_rt_target_time),
            *_pose_values_for_csv(controller_internal_command_pose),
            _format_age_value(controller_internal_command_age),
            *[f"{value:.12f}" for value in target_minus_current],
            *[f"{value:.12f}" for value in target_minus_command],
            *[f"{value:.12f}" for value in command_minus_current],
            process_state["controller_running"],
            server_flags.get("ros2_controller_active", ""),
            server_flags.get("command_connected", ""),
            server_flags.get("robot_state_fresh", ""),
            server_fields.get("target_seed_source", ""),
            server_fields.get("first_target_source", ""),
            server_fields.get("first_target_to_internal_command_norm", ""),
            server_fields.get("first_target_to_measured_norm", ""),
            server_fields.get("target_initialized_from_controller_command_pose", ""),
            server_fields.get("control_target_valid_for_motion", ""),
            server_fields.get("bottom_controller_targets_enabled", ""),
            server_fields.get("controller_enable_targets_success", ""),
            publish_block_reason or server_fields.get("publish_block_reason", ""),
            target_write or server_fields.get("write", ""),
            controller_fields.get("accept_targets", ""),
            controller_fields.get("target_accepted_count", ""),
            controller_fields.get("target_rejected_count", ""),
            controller_fields.get("last_target_reject_reason", ""),
            controller_fields.get("target_to_command_error", ""),
            controller_fields.get("target_to_measured_error", ""),
            controller_fields.get("command_to_measured_error", ""),
            controller_fields.get("activation_command_to_measured_norm", ""),
            controller_fields.get("seeded_command_to_measured_norm", ""),
            controller_fields.get("command_seed_source", ""),
            controller_fields.get("controller_update_period_s", ""),
            controller_fields.get("controller_update_overrun_count", ""),
            controller_fields.get("target_stream_primed", ""),
            controller_fields.get("has_target", ""),
            controller_fields.get("rt_has_target", ""),
            server_status,
            alert,
        ]

    def _process_record_state(self, now: float, last_current_pose_time: float) -> dict[str, int | str]:
        with self.process_lock:
            snapshot = dict(self.processes)
        result: dict[str, int | str] = {}
        for name in ("bringup", "controller", "teleop"):
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
        current_pose_fresh = (
            last_current_pose_time > 0.0 and now - last_current_pose_time <= 0.25
        )
        bringup_exit_code = result["bringup_exit_code"]
        if result["bringup_running"] == 1:
            machine_state = 1
        elif bringup_exit_code not in ("", 0):
            machine_state = 0
        else:
            machine_state = int(current_pose_fresh)
        result["machine_state"] = machine_state
        return result

    def _start_process(self, name: str, script_name: str) -> None:
        with self.process_lock:
            proc = self.processes.get(name)
        if proc is not None and proc.poll() is None:
            self.log_var.set(f"{name} already running, pid={proc.pid}")
            return

        script = PROJECT_DIR / "scripts" / script_name
        log_path = LOG_DIR / f"spacemouse_franka_{name}.log"
        log_file = log_path.open("ab", buffering=0)
        env = os.environ.copy()
        env.setdefault("SPACEMOUSE_FRANKA_TELEOP_DIR", str(PROJECT_DIR))
        try:
            proc = subprocess.Popen(
                [str(script)],
                cwd=str(PROJECT_DIR),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
        except Exception as exc:
            log_file.close()
            self.log_var.set(f"{name} start failed: {exc}")
            return
        log_file.close()
        with self.process_lock:
            self.processes[name] = proc
        self.log_var.set(f"started {name}, pid={proc.pid}, log={log_path}")

    def _clear_ros(self) -> None:
        threading.Thread(target=self._clear_ros_worker, daemon=True).start()

    def _clear_ros_worker(self) -> None:
        script = PROJECT_DIR / "scripts" / "clear_ros.sh"
        log_path = LOG_DIR / "spacemouse_franka_clear.log"
        with self.process_lock:
            processes = list(self.processes.items())
            self.processes.clear()
        for name, proc in processes:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except Exception:
                    pass
        with log_path.open("ab", buffering=0) as log_file:
            subprocess.run(
                [str(script)],
                cwd=str(PROJECT_DIR),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                env=os.environ.copy(),
                check=False,
            )
        self.log_var.set(f"clear complete, log={log_path}")

    def _schedule_refresh(self) -> None:
        self._refresh_values()
        self.root.after(50, self._schedule_refresh)

    def _refresh_values(self) -> None:
        with self.state.lock:
            spacemouse = self.state.spacemouse.copy()
            teleop_state = self.state.teleop_state
            fine_mode = self.state.fine_mode
            current_pose = None if self.state.current_pose is None else self.state.current_pose.copy()
            target_pose = None if self.state.target_pose is None else self.state.target_pose.copy()
            controller_internal_command_pose = (
                None
                if self.state.controller_internal_command_pose is None
                else self.state.controller_internal_command_pose.copy()
            )
            ros_status = self.state.ros_status
            server_status = self.state.server_status
            controller_target_status = self.state.controller_target_status
            alert = self.state.alert
            reset_status = self.state.reset_status
            last_current_pose_time = self.state.last_current_pose_time
            last_target_pose_time = self.state.last_target_pose_time
            last_controller_internal_command_time = self.state.last_controller_internal_command_time

        self.spacemouse_var.set(
            "x {:+.4f}\ny {:+.4f}\nz {:+.4f}\nroll {:+.4f}\npitch {:+.4f}\nyaw {:+.4f}\nstate {}\nfine {}".format(
                *spacemouse,
                teleop_state,
                "on" if fine_mode else "off",
            )
        )
        self.current_pose_var.set(format_pose(current_pose))
        self.target_pose_var.set(
            format_target_pose(target_pose, server_status, controller_target_status, last_target_pose_time)
        )
        self.controller_internal_command_var.set(
            format_stamped_pose(
                controller_internal_command_pose,
                last_controller_internal_command_time,
                "缺少 controller_internal_command_pose",
            )
        )
        self.pose_error_var.set(
            format_pose_errors(
                current_pose,
                target_pose,
                controller_internal_command_pose,
                last_target_pose_time,
                last_controller_internal_command_time,
            )
        )
        self.ros_status_var.set(ros_status)
        self.server_status_var.set(server_status)
        self.safety_status_var.set(
            _format_safety_status(server_status, controller_target_status, last_current_pose_time)
        )
        self.reset_status_var.set(reset_status)
        self.alert_var.set(alert)
        self.alert_label.configure(style="Ok.TLabel" if alert == "OK" else "Alert.TLabel")
        self._refresh_process_status()

    def _refresh_process_status(self) -> None:
        with self.process_lock:
            processes = dict(self.processes)
        for name, var in self.status_vars.items():
            proc = processes.get(name)
            if proc is None:
                var.set("stopped")
            elif proc.poll() is None:
                var.set(f"running pid={proc.pid}")
            else:
                var.set(f"exited code={proc.returncode}")

    def _on_close(self) -> None:
        if self.record_thread is not None and self.record_thread.is_alive():
            self.record_stop_event.set()
            self.record_thread.join(timeout=2.0)
        if self.ros_executor is not None:
            self.ros_executor.shutdown()
        self.root.destroy()


def pose_to_array(msg: PoseStamped) -> np.ndarray:
    return np.array(
        [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ],
        dtype=np.float64,
    )


def format_pose(pose: np.ndarray | None) -> str:
    if pose is None:
        return "waiting"
    return (
        "x {:+.5f}\ny {:+.5f}\nz {:+.5f}\n"
        "qx {:+.5f}\nqy {:+.5f}\nqz {:+.5f}\nqw {:+.5f}"
    ).format(*pose)


def format_target_pose(
    pose: np.ndarray | None,
    server_status: str,
    controller_target_status: str,
    last_target_pose_time: float,
) -> str:
    now = time.monotonic()
    target_age = now - last_target_pose_time if last_target_pose_time > 0.0 else float("nan")
    target_fresh = bool(last_target_pose_time > 0.0 and target_age <= 0.25)
    if pose is not None and target_fresh:
        return format_pose(pose)

    reason = _target_missing_reason(server_status, controller_target_status, target_age)
    age_text = _format_age_value(target_age)
    return f"缺少 target_pose\n原因: {reason}\ntarget_age_s {age_text}"


def format_stamped_pose(pose: np.ndarray | None, stamp: float, missing_text: str) -> str:
    now = time.monotonic()
    age = now - stamp if stamp > 0.0 else float("nan")
    fresh = bool(stamp > 0.0 and age <= 0.25)
    if pose is None:
        return f"{missing_text}\nage_s {_format_age_value(age)}"
    return f"{format_pose(pose)}\nage_s {_format_age_value(age)}\nfresh {_yes_no(fresh)}"


def format_pose_errors(
    current_pose: np.ndarray | None,
    target_pose: np.ndarray | None,
    controller_internal_command_pose: np.ndarray | None,
    last_target_pose_time: float,
    last_controller_internal_command_time: float,
) -> str:
    now = time.monotonic()
    target_fresh = bool(last_target_pose_time > 0.0 and now - last_target_pose_time <= 0.25)
    command_fresh = bool(
        last_controller_internal_command_time > 0.0
        and now - last_controller_internal_command_time <= 0.25
    )

    def line(label: str, value: np.ndarray | None) -> str:
        if value is None:
            return f"{label}: nan"
        xy = float(np.linalg.norm(value[:2]))
        norm = float(np.linalg.norm(value))
        return (
            f"{label}: x {value[0]:+.6f} y {value[1]:+.6f} z {value[2]:+.6f} "
            f"xy {xy:.6f} norm {norm:.6f}"
        )

    target_for_error = target_pose if target_fresh else None
    command_for_error = controller_internal_command_pose if command_fresh else None
    target_minus_current = (
        target_for_error[:3] - current_pose[:3]
        if target_for_error is not None and current_pose is not None
        else None
    )
    target_minus_command = (
        target_for_error[:3] - command_for_error[:3]
        if target_for_error is not None and command_for_error is not None
        else None
    )
    command_minus_current = (
        command_for_error[:3] - current_pose[:3]
        if command_for_error is not None and current_pose is not None
        else None
    )
    return "\n".join(
        [
            f"target_fresh: {_yes_no(target_fresh)}  command_fresh: {_yes_no(command_fresh)}",
            line("target-current", target_minus_current),
            line("target-command", target_minus_command),
            line("command-current", command_minus_current),
        ]
    )


def _target_missing_reason(
    server_status: str, controller_target_status: str, target_age: float
) -> str:
    if not server_status or server_status == "waiting":
        return "未启动或尚未收到 pose_action_server 状态"

    fields = _status_fields(server_status)
    controller_fields = _status_fields(controller_target_status)
    state = fields.get("state", "unknown")

    block_reason = _status_value_span(server_status, "block_reason", "tracking_block_reason")
    tracking_block = _status_value_span(
        server_status, "tracking_block_reason", "target_jump_rejected_reason"
    )
    target_jump = _status_value_span(
        server_status, "target_jump_rejected_reason", "target_write_reason"
    )
    publish_block = _status_value_span(
        server_status, "publish_block_reason", "target_initialized_from_controller_command_pose"
    )

    for reason in (target_jump, tracking_block, block_reason):
        if reason and reason.lower() not in ("none", "false"):
            return reason

    if state == "WAITING_FOR_MEASURED":
        return "等待 Franka measured pose 更新或 robot_state_fresh=false"
    if state == "WAITING_FOR_CONTROLLER":
        return "等待 Cartesian controller active"
    if state == "IDLE":
        if publish_block:
            return publish_block
        return "IDLE 中不发布 target，等待 deadman 上升沿"

    if publish_block:
        return publish_block
    if fields.get("target_initialized_from_controller_command_pose", "") == "False":
        return "target 尚未由 controller_internal_command_pose 初始化"
    if fields.get("control_target_valid_for_motion", "") == "False":
        return "control target 当前无效"
    if fields.get("bottom_controller_targets_enabled", "") == "False":
        return "底层 controller target stream 未开启"
    if controller_fields.get("accept_targets", "") == "False":
        reject = controller_fields.get("last_target_reject_reason", "")
        return f"底层 controller 未接受 target{(': ' + reject) if reject else ''}"
    if np.isfinite(target_age):
        return "target_pose topic 已超时，当前没有 fresh target"
    return "尚未收到 target_pose topic"


def math_nan() -> float:
    return float("nan")


def _age_for_csv(now: float, stamp: float) -> str:
    if stamp <= 0.0:
        return "nan"
    return f"{now - stamp:.9f}"


def _format_age_value(value: float) -> str:
    if not np.isfinite(value):
        return "nan"
    return f"{value:.9f}"


def _pose_values_for_csv(pose: np.ndarray | None) -> list[str]:
    if pose is None:
        return ["nan"] * len(POSE_FIELD_NAMES)
    return [f"{float(value):.12f}" for value in pose]


def _parse_server_status(status: str) -> dict[str, int | str]:
    fields = _status_fields(status)
    controller = fields.get("controller", "")
    return {
        "ros2_controller_active": int(controller == "active"),
        "command_connected": _bool_text_to_int(fields.get("command_connected", "")),
        "robot_state_fresh": _bool_text_to_int(fields.get("robot_state_fresh", "")),
    }


def _format_safety_status(
    status: str, controller_target_status: str, last_current_pose_time: float
) -> str:
    fields = _status_fields(status)
    controller_fields = _status_fields(controller_target_status)
    now = time.monotonic()
    current_age = now - last_current_pose_time if last_current_pose_time > 0.0 else float("nan")
    current_fresh = bool(last_current_pose_time > 0.0 and current_age <= 0.25)
    block_reason = _status_value_span(status, "block_reason", "tracking_block_reason")
    tracking_block = _status_value_span(status, "tracking_block_reason", "target_jump_rejected_reason")
    target_jump = _status_value_span(status, "target_jump_rejected_reason", "target_write_reason")
    publish_block = _status_value_span(
        status, "publish_block_reason", "target_initialized_from_controller_command_pose"
    )
    if not publish_block:
        publish_block = fields.get("publish_block_reason", "")
    lines = [
        f"current_pose_fresh: {_yes_no(current_fresh)} age={_format_age_value(current_age)}s",
        f"robot_state_fresh: {fields.get('robot_state_fresh', 'unknown')}",
        f"state: {fields.get('state', 'unknown')}  teleop: {fields.get('teleop', 'unknown')}",
        f"block: {block_reason or fields.get('block_reason', 'none') or 'none'}",
        f"tracking_block: {tracking_block or fields.get('tracking_block_reason', 'none') or 'none'}",
        f"target_jump: {target_jump or fields.get('target_jump_rejected_reason', 'none') or 'none'}",
        (
            "target gates: "
            f"init_from_command={fields.get('target_initialized_from_controller_command_pose', 'unknown')} "
            f"valid={fields.get('control_target_valid_for_motion', 'unknown')} "
            f"bottom={fields.get('bottom_controller_targets_enabled', 'unknown')} "
            f"enable={fields.get('controller_enable_targets_success', 'unknown')}"
        ),
        (
            "start continuity: "
            f"seed={fields.get('target_seed_source', 'unknown')} "
            f"cmd_age={fields.get('controller_internal_command_pose_age_s', 'nan')}s "
            f"cmd_meas_xy={fields.get('command_minus_measured_xy', 'nan')} "
            f"cmd_meas_z={fields.get('command_minus_measured_z', 'nan')} "
            f"limit={fields.get('tracking_start_tolerance', 'nan')}"
        ),
        (
            "target continuity: "
            f"target_cmd_xy={fields.get('target_minus_command_xy', 'nan')} "
            f"target_cmd_z={fields.get('target_minus_command_z', 'nan')}"
        ),
        (
            "publish: "
            f"count={fields.get('target_publish_count', '0')} "
            f"block={publish_block or 'none'}"
        ),
        (
            "controller target: "
            f"accept={controller_fields.get('accept_targets', 'unknown')} "
            f"accepted={controller_fields.get('target_accepted_count', '0')} "
            f"rejected={controller_fields.get('target_rejected_count', '0')} "
            f"reject={controller_fields.get('last_target_reject_reason', 'none')}"
        ),
        (
            "controller errors: "
            f"target_cmd={controller_fields.get('target_to_command_error', 'nan')} "
            f"target_meas={controller_fields.get('target_to_measured_error', 'nan')} "
            f"cmd_meas={controller_fields.get('command_to_measured_error', 'nan')}"
        ),
    ]
    return "\n".join(lines)


def _status_fields(status: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in status.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value
    return fields


def _yes_no(value: bool) -> str:
    return "yes" if value else "no"


def _status_value_span(status: str, key: str, next_key: str) -> str:
    start_marker = f"{key}="
    start = status.find(start_marker)
    if start < 0:
        return ""
    start += len(start_marker)
    end_marker = f" {next_key}="
    end = status.find(end_marker, start)
    if end < 0:
        end = len(status)
    return status[start:end].strip()


def _bool_text_to_int(value: str) -> int | str:
    if value == "True":
        return 1
    if value == "False":
        return 0
    return ""


def main() -> None:
    TeleopDashboardApp().run()


if __name__ == "__main__":
    main()
