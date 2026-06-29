#!/usr/bin/env python3
"""SpaceMouse acceleration intent -> flight-style Cartesian pose target server."""

from __future__ import annotations

import enum
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
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String

from .motion_shaping import (
    AxisMotionLimits,
    MotionShaper,
    MotionShapingConfig,
    StepDeltaLimits,
    TrackingErrorLimits,
    power_deadzone,
)


class ControlState(enum.Enum):
    WAITING_FOR_MEASURED = "WAITING_FOR_MEASURED"
    WAITING_FOR_CONTROLLER = "WAITING_FOR_CONTROLLER"
    IDLE = "IDLE"
    HUMAN_CONTROL = "HUMAN_CONTROL"
    BRAKE = "BRAKE"


class TargetManager:
    """Single public target write entry point for the pose action server."""

    def __init__(self, owner: "PoseActionServerNode") -> None:
        self.owner = owner

    def request_target(
        self,
        candidate: np.ndarray,
        reason: str,
        writer: str,
        *,
        allow_initial: bool = False,
        reject_on_jump: bool = True,
    ) -> bool:
        return self.owner._request_target_impl(
            candidate,
            reason,
            writer,
            allow_initial=allow_initial,
            reject_on_jump=reject_on_jump,
        )

    def clear_history(self, reason: str, writer: str) -> None:
        self.owner._clear_target_history(reason, writer)

    @property
    def current_target_pose(self) -> np.ndarray | None:
        return self.owner.target_pose


def get_double_array(node: Node, name: str) -> np.ndarray:
    return np.asarray(node.get_parameter(name).value, dtype=np.float64)


def pose_msg_to_array(msg: PoseStamped) -> np.ndarray:
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
    quat_norm = float(np.linalg.norm(quat))
    if quat_norm < 1e-9:
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    else:
        quat = quat / quat_norm
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


def wrench_norm(msg: WrenchStamped | None) -> tuple[float, float]:
    if msg is None:
        return 0.0, 0.0
    force = msg.wrench.force
    torque = msg.wrench.torque
    force_norm = math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z)
    torque_norm = math.sqrt(torque.x * torque.x + torque.y * torque.y + torque.z * torque.z)
    return force_norm, torque_norm


class PoseActionServerNode(Node):
    def __init__(self) -> None:
        super().__init__("spacemouse_franka_pose_action_server")

        self.declare_parameter("action_topic", "/spacemouse_franka_teleop/action")
        self.declare_parameter("deadman_topic", "/spacemouse_franka_teleop/deadman")
        self.declare_parameter("retreat_topic", "/spacemouse_franka_teleop/retreat")
        self.declare_parameter("fine_mode_topic", "/spacemouse_franka_teleop/fine_mode")
        self.declare_parameter("target_pose_topic", "/serl_cartesian_impedance_controller/target_pose")
        self.declare_parameter("current_pose_topic", "/franka_robot_state_broadcaster/current_pose")
        self.declare_parameter(
            "command_pose_topic", "/franka_robot_state_broadcaster/last_desired_pose"
        )
        self.declare_parameter(
            "controller_internal_command_pose_topic",
            "",
        )
        self.declare_parameter(
            "controller_raw_received_target_pose_topic",
            "",
        )
        self.declare_parameter(
            "controller_accepted_target_pose_topic",
            "",
        )
        self.declare_parameter(
            "controller_rt_target_pose_topic",
            "",
        )
        self.declare_parameter(
            "controller_target_status_topic",
            "",
        )
        self.declare_parameter("server_status_topic", "/spacemouse_franka_teleop/server_status")
        self.declare_parameter("reset_service_name", "/spacemouse_franka_pose_action_server/reset_to_current_pose")
        self.declare_parameter("publish_enabled", False)
        self.declare_parameter("force_zero_motion", False)
        self.declare_parameter("debug_csv_path", "/tmp/spacemouse_franka_target_trace.csv")
        self.declare_parameter(
            "wrench_topic", "/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame"
        )
        self.declare_parameter("joint_states_topic", "franka/joint_states")
        self.declare_parameter("controller_manager_service", "/controller_manager/list_controllers")
        self.declare_parameter("controller_name", "serl_cartesian_impedance_controller")
        self.declare_parameter(
            "controller_hold_service",
            "",
        )
        self.declare_parameter(
            "controller_clear_target_service",
            "",
        )
        self.declare_parameter(
            "controller_enable_targets_service",
            "",
        )
        self.declare_parameter("require_active_controller", True)
        self.declare_parameter("debug_simulation_mode", False)
        self.declare_parameter("fake_current_pose_when_debug", False)
        self.declare_parameter("frame_id", "base")
        self.declare_parameter("rate_hz", 1000.0)
        self.declare_parameter("log_rate_hz", 2.0)

        self.declare_parameter("control_scale", 1.0)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("translation_deadzone", 0.06)
        self.declare_parameter("rotation_deadzone", 0.10)
        self.declare_parameter("input_power", 3.0)
        self.declare_parameter("max_action_norm", 1.0)
        self.declare_parameter("command_timeout_s", 0.10)
        self.declare_parameter("robot_state_timeout_s", 0.10)
        self.declare_parameter("command_pose_timeout_s", 0.25)
        self.declare_parameter("tracking_start_tolerance", 0.0005)

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
        self.declare_parameter("dt_nominal", 0.001)
        self.declare_parameter("dt_max", 0.003)
        self.declare_parameter("delta_xy_max", 0.000010)
        self.declare_parameter("delta_z_up_max", 0.000010)
        self.declare_parameter("delta_z_down_max", 0.000003)
        self.declare_parameter("workspace_slowdown_distance", 0.030)
        self.declare_parameter("tracking_xy_error_max", 0.001)
        self.declare_parameter("tracking_z_error_max", 0.0005)
        self.declare_parameter("joint_position_margin", 0.2)
        self.declare_parameter(
            "joint_names",
            [
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
        )
        self.declare_parameter(
            "joint_position_lower",
            [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159],
        )
        self.declare_parameter(
            "joint_position_upper",
            [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159],
        )

        self.declare_parameter("max_force_n", 25.0)
        self.declare_parameter("max_torque_nm", 8.0)
        self.declare_parameter("workspace_low", [0.25, -0.20, 0.04])
        self.declare_parameter("workspace_high", [0.75, 0.25, 0.75])
        self.declare_parameter("axis_sign", [1.0, 1.0, 1.0])

        self.action_topic = self.get_parameter("action_topic").value
        self.deadman_topic = self.get_parameter("deadman_topic").value
        self.retreat_topic = self.get_parameter("retreat_topic").value
        self.fine_mode_topic = self.get_parameter("fine_mode_topic").value
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.current_pose_topic = self.get_parameter("current_pose_topic").value
        self.command_pose_topic = self.get_parameter("command_pose_topic").value
        self.controller_internal_command_pose_topic = self.get_parameter(
            "controller_internal_command_pose_topic"
        ).value
        self.controller_raw_received_target_pose_topic = self.get_parameter(
            "controller_raw_received_target_pose_topic"
        ).value
        self.controller_accepted_target_pose_topic = self.get_parameter(
            "controller_accepted_target_pose_topic"
        ).value
        self.controller_rt_target_pose_topic = self.get_parameter(
            "controller_rt_target_pose_topic"
        ).value
        self.controller_target_status_topic = self.get_parameter(
            "controller_target_status_topic"
        ).value
        self.server_status_topic = self.get_parameter("server_status_topic").value
        self.reset_service_name = self.get_parameter("reset_service_name").value
        self.publish_enabled = bool(self.get_parameter("publish_enabled").value)
        self.force_zero_motion = bool(self.get_parameter("force_zero_motion").value)
        self.debug_csv_path = str(self.get_parameter("debug_csv_path").value)
        self.wrench_topic = self.get_parameter("wrench_topic").value
        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.controller_manager_service = self.get_parameter("controller_manager_service").value
        self.controller_name = self.get_parameter("controller_name").value
        self.controller_hold_service = self.get_parameter("controller_hold_service").value
        self.controller_clear_target_service = self.get_parameter("controller_clear_target_service").value
        self.controller_enable_targets_service = self.get_parameter(
            "controller_enable_targets_service"
        ).value
        self.require_active_controller = bool(self.get_parameter("require_active_controller").value)
        self.debug_simulation_mode = bool(self.get_parameter("debug_simulation_mode").value)
        self.fake_current_pose_when_debug = bool(
            self.get_parameter("fake_current_pose_when_debug").value
        )
        if self.debug_simulation_mode:
            self.require_active_controller = False
        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.log_rate_hz = float(self.get_parameter("log_rate_hz").value)

        self.motion_shaper = self._make_motion_shaper()
        self.target_manager = TargetManager(self)
        self.command_timeout_s = max(0.02, float(self.get_parameter("command_timeout_s").value))
        self.robot_state_timeout_s = max(
            0.02, float(self.get_parameter("robot_state_timeout_s").value)
        )
        self.command_pose_timeout_s = max(
            0.02, float(self.get_parameter("command_pose_timeout_s").value)
        )
        self.tracking_start_tolerance = max(
            0.0, float(self.get_parameter("tracking_start_tolerance").value)
        )

        self.max_force_n = max(0.0, float(self.get_parameter("max_force_n").value))
        self.max_torque_nm = max(0.0, float(self.get_parameter("max_torque_nm").value))
        self.workspace_low = get_double_array(self, "workspace_low")
        self.workspace_high = get_double_array(self, "workspace_high")
        self.axis_sign = get_double_array(self, "axis_sign")
        self.tracking_error_limits = self._read_tracking_error_limits()
        self.joint_position_margin = max(
            0.0, float(self.get_parameter("joint_position_margin").value)
        )
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.joint_position_lower = get_double_array(self, "joint_position_lower")
        self.joint_position_upper = get_double_array(self, "joint_position_upper")

        self.measured_pose_from_franka: np.ndarray | None = None
        self.current_pose: np.ndarray | None = None
        self.command_pose: np.ndarray | None = None
        self.controller_internal_command_pose: np.ndarray | None = None
        self.controller_raw_received_target_pose: np.ndarray | None = None
        self.controller_accepted_target_pose: np.ndarray | None = None
        self.controller_rt_target_pose: np.ndarray | None = None
        self.target_pose: np.ndarray | None = None
        self.hold_pose: np.ndarray | None = None
        self.published_target_pose: np.ndarray | None = None
        self.previous_published_target_pose: np.ndarray | None = None
        self.candidate_target_pose: np.ndarray | None = None
        self.pre_guard_target_pose: np.ndarray | None = None
        self.post_guard_target_pose: np.ndarray | None = None
        self.last_wrench: WrenchStamped | None = None
        self.last_joint_state: JointState | None = None
        self.raw_input_before_axis = np.zeros(3, dtype=np.float64)
        self.raw_action = np.zeros(3, dtype=np.float64)
        self.last_action_time = 0.0
        self.last_pose_time = 0.0
        self.last_measured_pose_header_time = math.nan
        self.previous_measured_pose_header_time = math.nan
        self.last_measured_pose_header_change_time = 0.0
        self.measured_pose_source = "franka_measured_state"
        self.measured_pose_topic = self.current_pose_topic
        self.measured_pose_frame = self.frame_id
        self.measured_pose_update_count = 0
        self.robot_state_callback_count = 0
        self.last_robot_state_callback_count = 0
        self.last_robot_state_fresh_check_time = 0.0
        self.last_command_pose_time = 0.0
        self.last_controller_internal_command_pose_time = 0.0
        self.last_controller_raw_received_target_time = 0.0
        self.last_controller_accepted_target_time = 0.0
        self.last_controller_rt_target_time = 0.0
        self.controller_target_status = ""
        self.controller_accept_targets = ""
        self.controller_target_accepted_count = ""
        self.controller_target_rejected_count = ""
        self.controller_last_target_reject_reason = ""
        self.controller_target_stream_primed = ""
        self.controller_has_target = ""
        self.controller_rt_has_target = ""
        self.controller_target_to_command_error = math.nan
        self.controller_target_to_measured_error = math.nan
        self.controller_command_to_measured_error = math.nan
        self.controller_activation_command_to_measured_norm = math.nan
        self.controller_seeded_command_to_measured_norm = math.nan
        self.controller_command_seed_source = ""
        self.controller_update_period_s = math.nan
        self.controller_update_overrun_count = 0
        self.prev_trace_time = 0.0
        self.prev_trace_target_position: np.ndarray | None = None
        self.prev_trace_target_velocity: np.ndarray | None = None
        self.prev_trace_target_acceleration: np.ndarray | None = None
        self.prev_trace_internal_command_position: np.ndarray | None = None
        self.prev_trace_internal_command_velocity: np.ndarray | None = None
        self.prev_trace_internal_command_acceleration: np.ndarray | None = None
        self.teleop_state = ControlState.IDLE
        self.deadman_pressed = False
        self.previous_deadman_pressed = False
        self.retreat_requested = False
        self.server_state = ControlState.WAITING_FOR_MEASURED
        self.state_transition_reason = "startup"
        self.block_reason = "startup"
        self.tracking_block_reason = ""
        self.target_jump_rejected_reason = ""
        self.target_write_reason = "NONE"
        self.target_writer = "NONE"
        self.first_target_pose: np.ndarray | None = None
        self.first_target_source = "NONE"
        self.last_target_seed_source = "NONE"
        self.last_tracking_reference_source = "NONE"
        self.tracking_ref_type = "measured_pose_from_franka"
        self.final_delta_clamped = False
        self.control_target_valid_for_motion = False
        self.first_human_target_pending = False
        self.first_human_target_published = False
        self.last_step_result = None
        self.last_tracking_error_guard_triggered = False
        self.fine_mode = False
        self.last_timer_time = time.monotonic()
        self.last_log_time = 0.0
        self.last_frame_warn_time = 0.0
        self.controller_active = not self.require_active_controller
        self.previous_controller_active = self.controller_active
        self.controller_state = "unchecked"
        self.last_controller_check_time = 0.0
        self.controller_check_period_s = 1.0
        self.pending_controller_future = None
        self.pending_controller_hold_future = None
        self.pending_controller_clear_future = None
        self.pending_controller_enable_future = None
        self.last_controller_hold_request_time = 0.0
        self.last_controller_clear_request_time = 0.0
        self.last_controller_enable_request_time = 0.0
        self.target_initialized_from_controller_command_pose = False
        self.bottom_controller_targets_enabled = False
        self.last_controller_enable_targets_success = False
        self.target_publish_count = 0
        self.last_publish_time = math.nan
        self.last_target_publish_period_s = math.nan
        self.last_publish_reason = "NONE"
        self.last_publish_writer = "NONE"
        self.publish_block_reason = "startup"
        self.target_topic_publisher_count = 0
        self.target_topic_publisher_count_ok = True
        self.last_status_message = ""
        self.debug_file = None
        self.debug_writer = None
        self.debug_write_count = 0

        if self.debug_simulation_mode and self.fake_current_pose_when_debug:
            self.measured_pose_from_franka = np.array(
                [0.45, 0.0, 0.20, 0.0, 0.0, 0.0, 1.0], dtype=np.float64
            )
            self.current_pose = self.measured_pose_from_franka
            self.controller_internal_command_pose = self.measured_pose_from_franka.copy()
            self.last_pose_time = time.monotonic()
            self.last_controller_internal_command_pose_time = self.last_pose_time
            self.last_measured_pose_header_change_time = self.last_pose_time
            self.measured_pose_update_count = 1
            self.robot_state_callback_count = 1

        self._open_debug_csv()

        self.target_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.server_status_topic, 10)
        self.create_subscription(TwistStamped, self.action_topic, self._action_cb, 10)
        self.create_subscription(Bool, self.deadman_topic, self._deadman_cb, 10)
        self.create_subscription(Bool, self.retreat_topic, self._retreat_cb, 10)
        self.create_subscription(Bool, self.fine_mode_topic, self._fine_mode_cb, 10)
        self.create_subscription(
            PoseStamped, self.current_pose_topic, self._pose_cb, qos_profile_sensor_data
        )
        if self.command_pose_topic:
            self.create_subscription(
                PoseStamped,
                self.command_pose_topic,
                self._command_pose_cb,
                qos_profile_sensor_data,
            )
        if self.controller_internal_command_pose_topic:
            self.create_subscription(
                PoseStamped,
                self.controller_internal_command_pose_topic,
                self._controller_internal_command_pose_cb,
                qos_profile_sensor_data,
            )
        if self.controller_raw_received_target_pose_topic:
            self.create_subscription(
                PoseStamped,
                self.controller_raw_received_target_pose_topic,
                self._controller_raw_received_target_pose_cb,
                qos_profile_sensor_data,
            )
        if self.controller_accepted_target_pose_topic:
            self.create_subscription(
                PoseStamped,
                self.controller_accepted_target_pose_topic,
                self._controller_accepted_target_pose_cb,
                qos_profile_sensor_data,
            )
        if self.controller_rt_target_pose_topic:
            self.create_subscription(
                PoseStamped,
                self.controller_rt_target_pose_topic,
                self._controller_rt_target_pose_cb,
                qos_profile_sensor_data,
            )
        if self.controller_target_status_topic:
            self.create_subscription(
                String,
                self.controller_target_status_topic,
                self._controller_target_status_cb,
                10,
            )
        self.create_subscription(
            WrenchStamped, self.wrench_topic, self._wrench_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, self.joint_states_topic, self._joint_state_cb, qos_profile_sensor_data
        )
        self.controller_client = self.create_client(ListControllers, self.controller_manager_service)
        self.controller_hold_client = (
            self.create_client(Trigger, self.controller_hold_service)
            if self.controller_hold_service
            else None
        )
        self.controller_clear_client = (
            self.create_client(Trigger, self.controller_clear_target_service)
            if self.controller_clear_target_service
            else None
        )
        self.controller_enable_client = (
            self.create_client(Trigger, self.controller_enable_targets_service)
            if self.controller_enable_targets_service
            else None
        )
        self.reset_service = self.create_service(
            Trigger,
            self.reset_service_name,
            self._reset_service_cb,
        )
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)

        self.get_logger().info("SpaceMouse pose action server started")
        self.get_logger().info(f"  action_topic: {self.action_topic}")
        self.get_logger().info(f"  deadman_topic: {self.deadman_topic}")
        self.get_logger().info(f"  retreat_topic: {self.retreat_topic}")
        self.get_logger().info(f"  target_pose_topic: {self.target_pose_topic}")
        self.get_logger().info(f"  command_pose_topic: {self.command_pose_topic or '<disabled>'}")
        self.get_logger().info(
            "  controller_internal_command_pose_topic: "
            f"{self.controller_internal_command_pose_topic or '<disabled>'}"
        )
        self.get_logger().info(f"  publish_enabled: {self.publish_enabled}")
        self.get_logger().info(f"  debug_csv_path: {self.debug_csv_path or '<disabled>'}")
        self.get_logger().info(f"  server_status_topic: {self.server_status_topic}")
        self.get_logger().info(f"  reset_service: {self.reset_service_name}")
        self.get_logger().info(f"  controller: {self.controller_name}")
        self.get_logger().info(f"  controller_hold_service: {self.controller_hold_service}")
        self.get_logger().info(f"  controller_clear_target_service: {self.controller_clear_target_service}")
        self.get_logger().info(f"  controller_enable_targets_service: {self.controller_enable_targets_service}")
        self.get_logger().info(f"  rate_hz: {self.rate_hz:.1f}")
        self.get_logger().info(f"  control_scale: {self.motion_shaper.config.speed_scale:.3f}")
        self.get_logger().info("  shaping: acceleration intent, jerk limited, flight style")

    def _adopt_pose_frame(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            now = time.monotonic()
            if now - self.last_frame_warn_time > 2.0:
                self.last_frame_warn_time = now
                self.get_logger().warn(
                    f"adopting pose frame '{msg.header.frame_id}' for target publishing "
                    f"(was '{self.frame_id}')"
                )
            self.frame_id = msg.header.frame_id

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._adopt_pose_frame(msg)
        pose = pose_msg_to_array(msg)
        if not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:7]) < 1e-9:
            return
        self.measured_pose_from_franka = pose
        self.current_pose = self.measured_pose_from_franka
        now = time.monotonic()
        self.last_pose_time = now
        header_time = self._stamp_to_seconds(msg)
        self.last_measured_pose_header_time = header_time
        if not math.isfinite(header_time):
            self.last_measured_pose_header_change_time = now
        elif (
            not math.isfinite(self.previous_measured_pose_header_time)
            or abs(header_time - self.previous_measured_pose_header_time) > 1e-12
        ):
            self.previous_measured_pose_header_time = header_time
            self.last_measured_pose_header_change_time = now
        self.measured_pose_frame = msg.header.frame_id or self.frame_id
        self.measured_pose_update_count += 1
        self.robot_state_callback_count += 1

    def _command_pose_cb(self, msg: PoseStamped) -> None:
        self._adopt_pose_frame(msg)
        pose = pose_msg_to_array(msg)
        if not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:7]) < 1e-9:
            return
        self.command_pose = pose
        self.last_command_pose_time = time.monotonic()

    def _controller_internal_command_pose_cb(self, msg: PoseStamped) -> None:
        self._adopt_pose_frame(msg)
        pose = pose_msg_to_array(msg)
        if not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:7]) < 1e-9:
            return
        now = time.monotonic()
        if self.last_controller_internal_command_pose_time > 0.0:
            self.controller_update_period_s = now - self.last_controller_internal_command_pose_time
        self.controller_internal_command_pose = pose
        self.last_controller_internal_command_pose_time = now

    def _controller_raw_received_target_pose_cb(self, msg: PoseStamped) -> None:
        pose = pose_msg_to_array(msg)
        if not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:7]) < 1e-9:
            return
        self.controller_raw_received_target_pose = pose
        self.last_controller_raw_received_target_time = time.monotonic()

    def _controller_accepted_target_pose_cb(self, msg: PoseStamped) -> None:
        pose = pose_msg_to_array(msg)
        if not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:7]) < 1e-9:
            return
        self.controller_accepted_target_pose = pose
        self.last_controller_accepted_target_time = time.monotonic()

    def _controller_rt_target_pose_cb(self, msg: PoseStamped) -> None:
        pose = pose_msg_to_array(msg)
        if not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:7]) < 1e-9:
            return
        self.controller_rt_target_pose = pose
        self.last_controller_rt_target_time = time.monotonic()

    def _controller_target_status_cb(self, msg: String) -> None:
        self.controller_target_status = msg.data
        fields = self._parse_status_fields(msg.data)
        self.controller_accept_targets = fields.get("accept_targets", "")
        self.controller_target_accepted_count = fields.get("target_accepted_count", "")
        self.controller_target_rejected_count = fields.get("target_rejected_count", "")
        self.controller_last_target_reject_reason = fields.get("last_target_reject_reason", "")
        self.controller_target_stream_primed = fields.get("target_stream_primed", "")
        self.controller_has_target = fields.get("has_target", "")
        self.controller_rt_has_target = fields.get("rt_has_target", "")
        self.controller_target_to_command_error = self._parse_float_field(
            fields.get("target_to_command_error", "")
        )
        self.controller_target_to_measured_error = self._parse_float_field(
            fields.get("target_to_measured_error", "")
        )
        self.controller_command_to_measured_error = self._parse_float_field(
            fields.get("command_to_measured_error", "")
        )
        self.controller_activation_command_to_measured_norm = self._parse_float_field(
            fields.get("activation_command_to_measured_norm", "")
        )
        self.controller_seeded_command_to_measured_norm = self._parse_float_field(
            fields.get("seeded_command_to_measured_norm", "")
        )
        self.controller_command_seed_source = fields.get("command_seed_source", "")
        if "controller_update_period_s" in fields:
            self.controller_update_period_s = self._parse_float_field(
                fields.get("controller_update_period_s", "")
            )
        if "controller_update_overrun_count" in fields:
            try:
                self.controller_update_overrun_count = int(
                    fields.get("controller_update_overrun_count", self.controller_update_overrun_count)
                )
            except (TypeError, ValueError):
                pass

    @staticmethod
    def _parse_status_fields(status: str) -> dict[str, str]:
        fields: dict[str, str] = {}
        for token in status.split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            fields[key] = value
        return fields

    @staticmethod
    def _parse_float_field(value: str) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return math.nan

    @staticmethod
    def _stamp_to_seconds(msg: PoseStamped) -> float:
        stamp = msg.header.stamp
        total_nsec = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if total_nsec <= 0:
            return math.nan
        return float(total_nsec) * 1e-9

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self.last_wrench = msg

    def _joint_state_cb(self, msg: JointState) -> None:
        self.last_joint_state = msg

    def _reset_service_cb(self, request, response):
        _ = request
        now = time.monotonic()
        if not self._measured_pose_ready(now):
            response.success = False
            response.message = (
                f"measured_pose_from_franka unavailable/stale; reset rejected "
                f"age={self._measured_pose_age(now):.3f}s callbacks={self.robot_state_callback_count}"
            )
            return response
        self._request_controller_clear("MANUAL_RESET")
        self._request_controller_hold("MANUAL_RESET")
        self._clear_uninitialized_target_state("MANUAL_RESET_CLEAR")
        self.block_reason = ""
        self.tracking_block_reason = ""
        self.target_jump_rejected_reason = ""
        self.control_target_valid_for_motion = False
        self.raw_input_before_axis[:] = 0.0
        self.raw_action[:] = 0.0
        self.motion_shaper.reset()
        self.first_human_target_pending = False
        self.first_human_target_published = False
        self._clear_uninitialized_target_state("MANUAL_RESET_CLEAR")
        self.server_state = ControlState.IDLE
        self.teleop_state = ControlState.IDLE
        self.state_transition_reason = "manual reset cleared target; bottom controller holds continuous command"
        self._publish_status("manual reset completed")
        response.success = True
        response.message = "cleared target; bottom controller holds continuous internal command pose"
        return response

    def _deadman_cb(self, msg: Bool) -> None:
        next_deadman = bool(msg.data)
        if next_deadman == self.deadman_pressed:
            return
        now = time.monotonic()
        self.previous_deadman_pressed = self.deadman_pressed
        self.deadman_pressed = next_deadman
        if next_deadman:
            self._start_human_control_from_controller_command(now)
        else:
            self.teleop_state = ControlState.IDLE
            self.first_human_target_pending = False
            self.first_human_target_published = False
            self.state_transition_reason = "deadman released"
            if self.target_pose is not None and not self.motion_shaper.is_stopped():
                self.server_state = ControlState.BRAKE
            else:
                self._finish_brake_to_idle("deadman released; no brake needed")

    def _retreat_cb(self, msg: Bool) -> None:
        self.retreat_requested = bool(msg.data)
        if self.retreat_requested:
            self.block_reason = "retreat disabled in minimal controller"
            self.state_transition_reason = self.block_reason
        if not self.deadman_pressed:
            self.teleop_state = ControlState.IDLE

    def _fine_mode_cb(self, msg: Bool) -> None:
        next_fine = bool(msg.data)
        if next_fine != self.fine_mode:
            self.fine_mode = next_fine
            self.state_transition_reason = "fine mode changed"

    def _action_cb(self, msg: TwistStamped) -> None:
        action = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            dtype=np.float64,
        )
        rotation = np.array(
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(action)) or not np.all(np.isfinite(rotation)):
            return
        self.raw_input_before_axis = np.clip(action, -1.0, 1.0)
        self.raw_action = np.clip(self.raw_input_before_axis * self.axis_sign, -1.0, 1.0)
        self.last_action_time = time.monotonic()
        _ = self.motion_shaper.shape_rotation(rotation)

    def _refresh_runtime_flags(self, now: float) -> None:
        if (
            self.debug_simulation_mode
            and self.fake_current_pose_when_debug
            and self.measured_pose_from_franka is not None
        ):
            self.last_pose_time = now
            self.measured_pose_update_count += 1
            self.robot_state_callback_count += 1
            if self.command_pose is None:
                self.command_pose = self.measured_pose_from_franka.copy()
            if self.controller_internal_command_pose is None:
                self.controller_internal_command_pose = self.measured_pose_from_franka.copy()
            self.last_command_pose_time = now
            self.last_controller_internal_command_pose_time = now
        self.publish_enabled = bool(self.get_parameter("publish_enabled").value)
        self.force_zero_motion = bool(self.get_parameter("force_zero_motion").value)
        self.tracking_start_tolerance = max(
            0.0, float(self.get_parameter("tracking_start_tolerance").value)
        )
        self._update_controller_status(now)
        if self.controller_active and not self.previous_controller_active:
            self.state_transition_reason = "controller activated; idle without target"
            if self._measured_pose_ready(now):
                self.server_state = ControlState.IDLE
            else:
                self.server_state = ControlState.WAITING_FOR_MEASURED
            self._clear_uninitialized_target_state("CONTROLLER_ACTIVATED_CLEAR_TARGET")
        self.previous_controller_active = self.controller_active

    def _start_human_control_from_controller_command(self, now: float) -> bool:
        self.raw_input_before_axis[:] = 0.0
        self.raw_action[:] = 0.0
        self.motion_shaper.reset()
        self._update_target_topic_publisher_count()
        ok, reason = self._human_control_entry_ok(now, self._command_connected())
        if not ok:
            next_state = (
                ControlState.WAITING_FOR_MEASURED
                if "measured pose" in reason
                else ControlState.WAITING_FOR_CONTROLLER
                if "controller_active=false" in reason
                else ControlState.IDLE
            )
            self._block_motion(next_state, f"deadman blocked: {reason}")
            return False
        command_pose, source = self._controller_internal_command_reference_pose(now)
        if command_pose is None:
            self._block_motion(
                ControlState.IDLE,
                "deadman blocked: controller_internal_command_pose unavailable",
            )
            return False
        self._clear_uninitialized_target_state("DEADMAN_RISING_CLEAR_TARGET")
        self.last_target_seed_source = source
        self.first_target_pose = command_pose.copy()
        self.first_target_source = "controller_internal_command_pose"
        self.target_initialized_from_controller_command_pose = True
        self.control_target_valid_for_motion = True
        if not self.target_manager.request_target(
            command_pose,
            "HUMAN_CONTROL_FIRST_TARGET",
            f"deadman_cb:{source}",
            allow_initial=True,
            reject_on_jump=False,
        ):
            self._block_motion(ControlState.IDLE, "deadman blocked: first target write failed")
            return False
        self.server_state = ControlState.HUMAN_CONTROL
        self.teleop_state = ControlState.HUMAN_CONTROL
        self.state_transition_reason = "deadman pressed; first target is controller_internal_command_pose"
        self.block_reason = ""
        self.publish_block_reason = "first target pending: accept_targets=false"
        self.first_human_target_pending = True
        self.first_human_target_published = False
        self.bottom_controller_targets_enabled = False
        self.last_controller_enable_targets_success = False
        self._request_controller_enable_targets("HUMAN_CONTROL_ENABLE_TARGETS")
        return True

    def _finish_brake_to_idle(self, reason: str) -> None:
        self._request_controller_clear(reason)
        self._clear_uninitialized_target_state(reason)
        self.server_state = ControlState.IDLE
        self.teleop_state = ControlState.IDLE
        self.state_transition_reason = reason
        self.raw_input_before_axis[:] = 0.0
        self.raw_action[:] = 0.0
        self.motion_shaper.reset()
        self.first_human_target_pending = False
        self.first_human_target_published = False

    def _tick(self) -> None:
        now = time.monotonic()
        raw_dt = now - self.last_timer_time
        self.last_timer_time = now
        self._refresh_runtime_flags(now)

        if self.measured_pose_from_franka is None:
            self._block_motion(ControlState.WAITING_FOR_MEASURED, "waiting for measured_pose_from_franka")
            self._log_throttled(now, "waiting for measured_pose_from_franka")
            self._write_debug_row(now, "waiting for measured_pose_from_franka", None)
            return

        if not self._robot_state_fresh(now):
            self._block_motion(
                ControlState.WAITING_FOR_MEASURED,
                "robot state stale",
                clear_target_history=True,
            )
            self._log_throttled(now, "robot state stale")
            self._write_debug_row(now, "robot state stale", None)
            return

        if self.server_state == ControlState.WAITING_FOR_MEASURED:
            self.server_state = ControlState.IDLE
            self.state_transition_reason = "measured pose fresh; idle without target"

        if (
            self.require_active_controller
            and not self.controller_active
            and self.controller_state != "unchecked"
        ):
            self._block_motion(
                ControlState.WAITING_FOR_CONTROLLER,
                f"controller not active: {self.controller_state}",
            )
            self._log_throttled(now, self.block_reason)
            self._write_debug_row(now, self.block_reason, None)
            return

        self._update_target_topic_publisher_count()
        if not self.target_topic_publisher_count_ok:
            self._block_motion(
                ControlState.IDLE,
                f"target topic publisher count invalid: "
                f"{self.target_topic_publisher_count} publishers on {self.target_pose_topic}",
            )
            self._log_throttled(now, self.block_reason)
            self._write_debug_row(now, self.block_reason, None)
            return

        if self.first_human_target_pending:
            self.raw_input_before_axis[:] = 0.0
            self.raw_action[:] = 0.0
            self.motion_shaper.reset()
            if not self.bottom_controller_targets_enabled:
                self._request_controller_enable_targets("HUMAN_CONTROL_ENABLE_TARGETS")
                note = "waiting for controller target acceptance before first target"
                self.publish_block_reason = "first target pending: accept_targets=false"
                self._log_throttled(now, note)
                self._write_debug_row(now, note, None)
                return
            self._publish_target_if_ready()
            self.first_human_target_pending = False
            self.first_human_target_published = True
            note = "first target published at controller_internal_command_pose"
            self._log_throttled(now, note)
            self._write_debug_row(now, note, None)
            return

        if self.server_state == ControlState.BRAKE:
            self._brake_and_publish(raw_dt)
            if self.motion_shaper.is_stopped():
                self._finish_brake_to_idle("brake finished")
                self._log_throttled(now, "brake finished; target cleared")
                self._write_debug_row(now, "brake finished; target cleared", self.last_step_result)
                return
            self._log_throttled(now, "brake")
            self._write_debug_row(now, "brake", self.last_step_result)
            return

        if self.server_state != ControlState.HUMAN_CONTROL:
            self._log_throttled(now, "idle; no target publish")
            self._write_debug_row(now, "idle; no target publish", None)
            return

        if not self.deadman_pressed:
            self.server_state = ControlState.BRAKE
            self.state_transition_reason = "deadman released; brake"
            return

        entry_ok, entry_note = self._human_control_entry_ok(now, self._command_connected())
        if not entry_ok:
            self._block_motion(ControlState.IDLE, f"human control blocked: {entry_note}")
            self._log_throttled(now, self.block_reason)
            self._write_debug_row(now, self.block_reason, None)
            return

        if self.target_pose is None:
            self._block_motion(ControlState.IDLE, "target_pose missing in HUMAN_CONTROL")
            self._log_throttled(now, self.block_reason)
            self._write_debug_row(now, self.block_reason, None)
            return

        self._refresh_motion_shaping_config()
        force_norm, torque_norm = wrench_norm(self.last_wrench)
        joint_margin_ok, joint_note = self._joint_margin_ok()
        note_parts = ["human acceleration control"]
        if force_norm > self.max_force_n or torque_norm > self.max_torque_nm:
            note_parts.append(f"force guard logged force={force_norm:.2f} torque={torque_norm:.2f}")
        if not joint_margin_ok:
            note_parts.append(joint_note)

        command_fresh = now - self.last_action_time <= self.command_timeout_s
        action = (
            np.zeros(3, dtype=np.float64)
            if self.force_zero_motion or not command_fresh
            else self.raw_action
        )
        step_result = self.motion_shaper.step(
            action,
            not self.force_zero_motion and command_fresh,
            self.fine_mode,
            raw_dt,
            self.target_pose[:3],
            self.workspace_low,
            self.workspace_high,
        )
        next_target = self.target_pose.copy()
        next_target[:3] += step_result.delta_position
        next_target[3:7] = self.target_pose[3:7]
        if not self.target_manager.request_target(next_target, "HUMAN_INTEGRATION", "control_loop"):
            self.last_step_result = step_result
            self._log_throttled(now, "target write rejected")
            self._write_debug_row(now, "target write rejected", step_result)
            return
        self.last_step_result = step_result
        self.last_tracking_error_guard_triggered = False
        self._publish_target_if_ready()
        if step_result.stale_dt:
            note_parts.append("stale dt -> nominal")
        if not command_fresh:
            note_parts.append("action timeout -> zero input")
        note = "; ".join(note_parts)
        self._log_throttled(now, note)
        self._write_debug_row(now, note, step_result)

    def _clear_uninitialized_target_state(self, reason: str) -> None:
        self.target_pose = None
        self.hold_pose = None
        self.published_target_pose = None
        self.previous_published_target_pose = np.full(7, np.nan, dtype=np.float64)
        self.candidate_target_pose = None
        self.pre_guard_target_pose = None
        self.post_guard_target_pose = None
        self.target_initialized_from_controller_command_pose = False
        self.bottom_controller_targets_enabled = False
        self.control_target_valid_for_motion = False
        self.first_target_pose = None
        self.first_target_source = "NONE"
        self.final_delta_clamped = False
        self.target_write_reason = reason
        self.target_writer = "control_loop"
        self.publish_block_reason = reason

    def _controller_internal_command_reference_pose(
        self, now: float
    ) -> tuple[np.ndarray | None, str]:
        if (
            self.controller_internal_command_pose is not None
            and self.last_controller_internal_command_pose_time > 0.0
            and now - self.last_controller_internal_command_pose_time <= self.command_pose_timeout_s
        ):
            return self.controller_internal_command_pose.copy(), "controller_internal_command_pose"
        return None, "NONE"

    @staticmethod
    def _nlerp_quaternion(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
        a = np.asarray(q0, dtype=np.float64)
        b = np.asarray(q1, dtype=np.float64)
        if np.linalg.norm(a) < 1e-9:
            a = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        if np.linalg.norm(b) < 1e-9:
            b = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        a = a / np.linalg.norm(a)
        b = b / np.linalg.norm(b)
        if float(np.dot(a, b)) < 0.0:
            b = -b
        q = (1.0 - alpha) * a + alpha * b
        norm = float(np.linalg.norm(q))
        if norm < 1e-9:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        return q / norm

    def _block_motion(
        self,
        state: ControlState,
        reason: str,
        *,
        clear_target_history: bool = False,
        disable_bottom_targets: bool = True,
    ) -> None:
        self.server_state = state
        self.state_transition_reason = reason
        self.block_reason = reason
        self.publish_block_reason = reason
        if state in (
            ControlState.WAITING_FOR_MEASURED,
            ControlState.WAITING_FOR_CONTROLLER,
            ControlState.IDLE,
        ):
            self.teleop_state = ControlState.IDLE
        self.raw_input_before_axis[:] = 0.0
        self.raw_action[:] = 0.0
        self.motion_shaper.reset()
        self.first_human_target_pending = False
        self.first_human_target_published = False
        if disable_bottom_targets:
            self.bottom_controller_targets_enabled = False
            self.last_controller_enable_targets_success = False
            self.control_target_valid_for_motion = False
            self._request_controller_clear(reason)
        if clear_target_history:
            self._clear_uninitialized_target_state(reason)

    def _publish_target_if_ready(self) -> None:
        if self.target_pose is None:
            self.publish_block_reason = "target_pose missing"
            return
        if not self.publish_enabled:
            self.publish_block_reason = "publish_enabled=false"
            return
        now = time.monotonic()
        if not self._robot_state_fresh(now):
            self.publish_block_reason = "robot_state_fresh=false"
            return
        if not self._measured_pose_valid():
            self.publish_block_reason = "measured_pose_valid=false"
            return
        if not self.target_initialized_from_controller_command_pose:
            self.publish_block_reason = "target not initialized from controller_internal_command_pose"
            return
        if not self.control_target_valid_for_motion:
            self.publish_block_reason = "control_target_valid_for_motion=false"
            return
        if self.require_active_controller and not self.controller_active:
            self.publish_block_reason = "controller_active=false"
            return
        if self.server_state not in (
            ControlState.HUMAN_CONTROL,
            ControlState.BRAKE,
        ):
            self.publish_block_reason = f"server_state={self.server_state.value}"
            return
        if not self.bottom_controller_targets_enabled:
            self.publish_block_reason = "bottom_controller_targets_enabled=false"
            return
        self.previous_published_target_pose = (
            self.published_target_pose.copy()
            if self.published_target_pose is not None
            else np.full(7, np.nan, dtype=np.float64)
        )
        self.published_target_pose = self.target_pose.copy()
        self.target_pub.publish(array_to_pose_msg(self.published_target_pose, self.frame_id, self))
        self.target_publish_count += 1
        if math.isfinite(self.last_publish_time) and self.last_publish_time > 0.0:
            self.last_target_publish_period_s = now - self.last_publish_time
        else:
            self.last_target_publish_period_s = math.nan
        self.last_publish_time = now
        self.last_publish_reason = self.target_write_reason
        self.last_publish_writer = self.target_writer
        self.publish_block_reason = "published"

    def _update_target_topic_publisher_count(self) -> None:
        publisher_info = self.get_publishers_info_by_topic(self.target_pose_topic)
        own_name = self.get_name()
        own_namespace = self.get_namespace()
        external_publishers = [
            info
            for info in publisher_info
            if not (
                getattr(info, "node_name", "") == own_name
                and getattr(info, "node_namespace", "") == own_namespace
            )
        ]
        self.target_topic_publisher_count = len(publisher_info)
        self.target_topic_publisher_count_ok = len(external_publishers) == 0

    def _brake_and_publish(self, dt: float) -> None:
        if self.target_pose is None or self.measured_pose_from_franka is None:
            return
        step_result = self._brake_internal(dt)
        if step_result is None:
            return
        next_target = self.target_pose.copy()
        next_target[:3] += step_result.delta_position
        next_target[3:7] = self.target_pose[3:7]
        if not self.target_manager.request_target(next_target, "BRAKE_INTEGRATION", "control_loop"):
            self.last_step_result = step_result
            return
        self.last_step_result = step_result
        self._publish_target_if_ready()

    def _brake_internal(self, dt: float):
        if self.target_pose is None:
            return None
        self._refresh_motion_shaping_config()
        return self.motion_shaper.step(
            np.zeros(3, dtype=np.float64),
            False,
            self.fine_mode,
            dt,
            self.target_pose[:3],
            self.workspace_low,
            self.workspace_high,
        )

    def _make_motion_shaper(self) -> MotionShaper:
        return MotionShaper(self._read_motion_shaping_config())

    def _read_axis_motion_limits(self, prefix: str) -> AxisMotionLimits:
        return AxisMotionLimits(
            v_xy=max(0.0, float(self.get_parameter(f"{prefix}_v_xy_max").value)),
            v_z_up=max(0.0, float(self.get_parameter(f"{prefix}_v_z_up_max").value)),
            v_z_down=max(0.0, float(self.get_parameter(f"{prefix}_v_z_down_max").value)),
            a_xy=max(0.0, float(self.get_parameter(f"{prefix}_a_xy_max").value)),
            a_z_up=max(0.0, float(self.get_parameter(f"{prefix}_a_z_up_max").value)),
            a_z_down=max(0.0, float(self.get_parameter(f"{prefix}_a_z_down_max").value)),
            j_xy=max(0.0, float(self.get_parameter(f"{prefix}_j_xy_max").value)),
            j_z_up=max(0.0, float(self.get_parameter(f"{prefix}_j_z_up_max").value)),
            j_z_down=max(0.0, float(self.get_parameter(f"{prefix}_j_z_down_max").value)),
        )

    def _read_motion_shaping_config(self) -> MotionShapingConfig:
        control_scale = self._effective_control_scale()
        return MotionShapingConfig(
            speed_scale=control_scale,
            translation_deadzone=max(
                0.0, float(self.get_parameter("translation_deadzone").value)
            ),
            rotation_deadzone=max(0.0, float(self.get_parameter("rotation_deadzone").value)),
            input_power=max(1.0, float(self.get_parameter("input_power").value)),
            max_action_norm=max(1e-6, float(self.get_parameter("max_action_norm").value)),
            coarse_limits=self._read_axis_motion_limits("coarse"),
            fine_limits=self._read_axis_motion_limits("fine"),
            d_move=max(0.0, float(self.get_parameter("d_move").value)),
            d_stop=max(0.0, float(self.get_parameter("d_stop").value)),
            dt_nominal=max(1e-6, float(self.get_parameter("dt_nominal").value)),
            dt_max=max(1e-6, float(self.get_parameter("dt_max").value)),
            delta_limits=StepDeltaLimits(
                xy=max(0.0, float(self.get_parameter("delta_xy_max").value)),
                z_up=max(0.0, float(self.get_parameter("delta_z_up_max").value)),
                z_down=max(0.0, float(self.get_parameter("delta_z_down_max").value)),
            ),
            workspace_slowdown_distance=max(
                1e-6, float(self.get_parameter("workspace_slowdown_distance").value)
            ),
        )

    def _effective_control_scale(self) -> float:
        control_scale = max(0.0, float(self.get_parameter("control_scale").value))
        speed_scale = max(0.0, float(self.get_parameter("speed_scale").value))
        return control_scale * speed_scale

    def _refresh_motion_shaping_config(self) -> None:
        self.motion_shaper.config = self._read_motion_shaping_config()
        self.tracking_error_limits = self._read_tracking_error_limits()
        self.joint_position_margin = max(
            0.0, float(self.get_parameter("joint_position_margin").value)
        )
        self.robot_state_timeout_s = max(
            0.02, float(self.get_parameter("robot_state_timeout_s").value)
        )
        self.command_pose_timeout_s = max(
            0.02, float(self.get_parameter("command_pose_timeout_s").value)
        )
        self.tracking_start_tolerance = max(
            0.0, float(self.get_parameter("tracking_start_tolerance").value)
        )

    def _read_tracking_error_limits(self) -> TrackingErrorLimits:
        return TrackingErrorLimits(
            xy=max(0.0, float(self.get_parameter("tracking_xy_error_max").value)),
            z=max(0.0, float(self.get_parameter("tracking_z_error_max").value)),
        )

    def _tracking_error_ok(self) -> tuple[bool, str]:
        if self.measured_pose_from_franka is None or self.target_pose is None:
            return True, "tracking unchecked"
        reference_pose, source = self._tracking_reference_pose(time.monotonic())
        if reference_pose is None:
            return True, "tracking unchecked"
        self.last_tracking_reference_source = source
        self.tracking_ref_type = source
        error = self.target_pose[:3] - reference_pose[:3]
        xy_error = float(np.linalg.norm(error[:2]))
        z_error = abs(float(error[2]))
        if xy_error > self.tracking_error_limits.xy or z_error > self.tracking_error_limits.z:
            return False, f"tracking guard {source} xy={xy_error:.5f} z={z_error:.5f}"
        return True, f"tracking ok {source}"

    def _joint_margin_ok(self) -> tuple[bool, str]:
        if self.last_joint_state is None:
            return True, "joint margin unchecked"
        if len(self.joint_names) != len(self.joint_position_lower) or len(self.joint_names) != len(
            self.joint_position_upper
        ):
            return False, "joint margin config invalid"
        position_by_name = {
            name: position
            for name, position in zip(self.last_joint_state.name, self.last_joint_state.position)
        }
        for i, name in enumerate(self.joint_names):
            if name not in position_by_name:
                continue
            position = float(position_by_name[name])
            lower_margin = position - float(self.joint_position_lower[i])
            upper_margin = float(self.joint_position_upper[i]) - position
            if min(lower_margin, upper_margin) < self.joint_position_margin:
                return False, f"joint margin guard {name} margin={min(lower_margin, upper_margin):.3f}"
        return True, "joint margin ok"

    def _robot_state_fresh(self, now: float) -> bool:
        if self.measured_pose_from_franka is None or self.last_pose_time <= 0.0:
            return False
        if now - self.last_pose_time > self.robot_state_timeout_s:
            return False
        if (
            math.isfinite(self.last_measured_pose_header_time)
            and self.last_measured_pose_header_change_time > 0.0
            and now - self.last_measured_pose_header_change_time > self.robot_state_timeout_s
        ):
            return False
        return True

    def _measured_pose_ready(self, now: float) -> bool:
        return self._robot_state_fresh(now) and self._measured_pose_valid()

    def _measured_pose_valid(self) -> bool:
        pose = self.measured_pose_from_franka
        return bool(
            pose is not None and np.all(np.isfinite(pose)) and np.linalg.norm(pose[3:7]) >= 1e-9
        )

    def _command_connected(self) -> bool:
        if not self.publish_enabled:
            return True
        if not self.require_active_controller:
            return True
        return self.target_pub.get_subscription_count() > 0

    def _target_error_to_measured(self) -> tuple[float, float, np.ndarray]:
        if self.target_pose is None or self.measured_pose_from_franka is None:
            return math.inf, math.inf, np.full(3, np.nan)
        error = self.target_pose[:3] - self.measured_pose_from_franka[:3]
        return float(np.linalg.norm(error[:2])), abs(float(error[2])), error

    def _human_control_entry_ok(
        self, now: float, command_connected: bool
    ) -> tuple[bool, str]:
        controller_ready = (not self.require_active_controller) or self.controller_active
        if not controller_ready:
            return False, "controller_active=false"
        if not self._measured_pose_ready(now):
            return False, (
                f"measured pose not fresh age={self._measured_pose_age(now):.6f}s "
                f"callbacks={self.robot_state_callback_count}"
            )
        if not command_connected:
            return False, "target command topic has no controller subscription"
        if not self.target_topic_publisher_count_ok:
            return False, f"target topic publisher count={self.target_topic_publisher_count}"
        command_reference, command_source = self._controller_internal_command_reference_pose(now)
        if command_reference is None:
            return False, "controller_internal_command_pose unavailable"
        if self.measured_pose_from_franka is None:
            return False, "measured pose unavailable"
        command_error = command_reference[:3] - self.measured_pose_from_franka[:3]
        xy_error = float(np.linalg.norm(command_error[:2]))
        z_error = abs(float(command_error[2]))
        threshold = max(0.0, self.tracking_start_tolerance)
        if xy_error > threshold or z_error > threshold:
            return False, (
                f"controller_internal_command too far from measured source={command_source} "
                f"xy={xy_error:.6f} z={z_error:.6f} threshold={threshold:.6f}"
            )
        return True, "ok"

    def _measured_pose_age(self, now: float) -> float:
        return now - self.last_pose_time if self.last_pose_time > 0.0 else math.nan

    def _measured_pose_header_age(self, now: float) -> float:
        return (
            now - self.last_measured_pose_header_change_time
            if self.last_measured_pose_header_change_time > 0.0
            else math.nan
        )

    def _command_pose_fresh(self, now: float) -> bool:
        return (
            self.command_pose is not None
            and self.last_command_pose_time > 0.0
            and now - self.last_command_pose_time <= self.command_pose_timeout_s
        )

    def _tracking_reference_pose(self, now: float) -> tuple[np.ndarray | None, str]:
        _ = now
        if self.measured_pose_from_franka is not None:
            return self.measured_pose_from_franka.copy(), "measured_pose_from_franka"
        return None, "NONE"

    def _request_target_impl(
        self,
        candidate: np.ndarray,
        reason: str,
        writer: str,
        *,
        allow_initial: bool = False,
        reject_on_jump: bool = True,
    ) -> bool:
        candidate = np.asarray(candidate, dtype=np.float64).copy()
        self.candidate_target_pose = candidate.copy()
        previous = self.published_target_pose if self.published_target_pose is not None else self.target_pose
        self.previous_published_target_pose = (
            self.published_target_pose.copy()
            if self.published_target_pose is not None
            else np.full(7, np.nan, dtype=np.float64)
        )
        self.pre_guard_target_pose = previous.copy() if previous is not None else np.full(7, np.nan)
        self.post_guard_target_pose = candidate.copy()
        self.final_delta_clamped = False
        self.target_jump_rejected_reason = ""

        if not np.all(np.isfinite(candidate)) or np.linalg.norm(candidate[3:7]) < 1e-9:
            note = f"reject non-finite target reason={reason}"
            self._block_motion(ControlState.IDLE, note, disable_bottom_targets=True)
            return False

        if previous is None:
            if allow_initial or self.target_pose is None:
                self._write_target_pose(candidate, reason, writer)
                self.hold_pose = self.target_pose.copy()
                return True
            note = f"reject target without previous reason={reason}"
            self._block_motion(ControlState.IDLE, note, disable_bottom_targets=True)
            return False

        delta = candidate[:3] - previous[:3]
        limits = self._target_delta_limits(delta)
        exceeded = np.abs(delta) > limits + 1e-12
        if bool(np.any(exceeded)):
            self.final_delta_clamped = True
            clamped = previous.copy()
            clamped[:3] = previous[:3] + np.clip(delta, -limits, limits)
            clamped[3:7] = previous[3:7]
            self.post_guard_target_pose = clamped
            if not reject_on_jump:
                self.target_jump_rejected_reason = ""
                self._write_target_pose(clamped, reason, writer)
                return True
            note = (
                f"target jump rejected reason={reason} "
                f"delta={np.array2string(delta, precision=9, suppress_small=False)} "
                f"limits={np.array2string(limits, precision=9, suppress_small=False)}"
            )
            self.target_jump_rejected_reason = note
            self._block_motion(
                ControlState.IDLE,
                note,
                disable_bottom_targets=True,
            )
            self.target_write_reason = reason
            self.target_writer = writer
            return False

        self._write_target_pose(candidate, reason, writer)
        return True

    def _clear_target_history(self, reason: str, writer: str) -> None:
        self.target_pose = None
        self.published_target_pose = None
        self.target_initialized_from_controller_command_pose = False
        self.bottom_controller_targets_enabled = False
        self.control_target_valid_for_motion = False
        self.previous_published_target_pose = np.full(7, np.nan, dtype=np.float64)
        self.pre_guard_target_pose = np.full(7, np.nan, dtype=np.float64)
        self.post_guard_target_pose = np.full(7, np.nan, dtype=np.float64)
        self.candidate_target_pose = None
        self.first_target_pose = None
        self.first_target_source = "NONE"
        self.final_delta_clamped = False
        self.target_jump_rejected_reason = ""
        self.target_write_reason = f"{reason}_CLEAR_ONLY"
        self.target_writer = writer

    def _target_delta_limits(self, delta: np.ndarray) -> np.ndarray:
        config = self._read_motion_shaping_config()
        return np.array(
            [
                config.delta_limits.xy,
                config.delta_limits.xy,
                config.delta_limits.z_up if float(delta[2]) >= 0.0 else config.delta_limits.z_down,
            ],
            dtype=np.float64,
        )

    def _request_controller_hold(self, reason: str) -> None:
        now = time.monotonic()
        self.bottom_controller_targets_enabled = False
        self.control_target_valid_for_motion = False
        if now - self.last_controller_hold_request_time < 0.25:
            return
        if self.pending_controller_hold_future is not None:
            return
        self.last_controller_hold_request_time = now
        self.last_controller_enable_targets_success = False
        if self.controller_hold_client is None:
            return
        if not self.controller_hold_client.service_is_ready():
            return
        self.pending_controller_hold_future = self.controller_hold_client.call_async(Trigger.Request())
        self.pending_controller_hold_future.add_done_callback(
            lambda future: self._controller_hold_done_cb(future, reason)
        )

    def _request_controller_clear(self, reason: str) -> None:
        now = time.monotonic()
        self.bottom_controller_targets_enabled = False
        self.control_target_valid_for_motion = False
        if now - self.last_controller_clear_request_time < 0.25:
            return
        if self.pending_controller_clear_future is not None:
            return
        self.last_controller_clear_request_time = now
        self.last_controller_enable_targets_success = False
        if self.controller_clear_client is None:
            return
        if not self.controller_clear_client.service_is_ready():
            return
        self.pending_controller_clear_future = self.controller_clear_client.call_async(
            Trigger.Request()
        )
        self.pending_controller_clear_future.add_done_callback(
            lambda future: self._controller_clear_done_cb(future, reason)
        )

    def _request_controller_enable_targets(self, reason: str) -> None:
        now = time.monotonic()
        self.bottom_controller_targets_enabled = False
        if now - self.last_controller_enable_request_time < 0.25:
            return
        if self.pending_controller_enable_future is not None:
            return
        self.last_controller_enable_request_time = now
        self.last_controller_enable_targets_success = False
        if self.controller_enable_client is None:
            self.publish_block_reason = "controller enable_targets service disabled"
            return
        if not self.controller_enable_client.service_is_ready():
            self.publish_block_reason = "controller enable_targets service not ready"
            return
        self.pending_controller_enable_future = self.controller_enable_client.call_async(
            Trigger.Request()
        )
        self.pending_controller_enable_future.add_done_callback(
            lambda future: self._controller_enable_done_cb(future, reason)
        )

    def _controller_hold_done_cb(self, future, reason: str) -> None:
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn(
                    f"controller hold_current rejected during {reason}: {result.message}"
                )
        except Exception as exc:
            self.get_logger().warn(f"controller hold_current failed during {reason}: {exc}")
        finally:
            self.pending_controller_hold_future = None

    def _controller_clear_done_cb(self, future, reason: str) -> None:
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn(
                    f"controller clear_target rejected during {reason}: {result.message}"
                )
        except Exception as exc:
            self.get_logger().warn(f"controller clear_target failed during {reason}: {exc}")
        finally:
            self.pending_controller_clear_future = None

    def _controller_enable_done_cb(self, future, reason: str) -> None:
        try:
            result = future.result()
            if not result.success:
                self.bottom_controller_targets_enabled = False
                self.last_controller_enable_targets_success = False
                self.get_logger().warn(
                    f"controller enable_targets rejected during {reason}: {result.message}"
                )
            else:
                enable_still_allowed = (
                    self.server_state
                    in (ControlState.HUMAN_CONTROL, ControlState.BRAKE)
                    and self.control_target_valid_for_motion
                    and self.target_initialized_from_controller_command_pose
                )
                if enable_still_allowed:
                    self.bottom_controller_targets_enabled = True
                    self.last_controller_enable_targets_success = True
                    self.publish_block_reason = "bottom controller target acceptance enabled"
                    if self.first_human_target_pending:
                        self._publish_target_if_ready()
                        if self.publish_block_reason == "published":
                            self.first_human_target_pending = False
                            self.first_human_target_published = True
                else:
                    self.bottom_controller_targets_enabled = False
                    self.last_controller_enable_targets_success = False
                    self.publish_block_reason = "stale enable_targets response ignored"
                    self._request_controller_clear("stale enable_targets response ignored")
        except Exception as exc:
            self.bottom_controller_targets_enabled = False
            self.last_controller_enable_targets_success = False
            self.get_logger().warn(f"controller enable_targets failed during {reason}: {exc}")
        finally:
            self.pending_controller_enable_future = None

    def _write_target_pose(self, pose: np.ndarray, reason: str, writer: str) -> None:
        self.target_pose = np.asarray(pose, dtype=np.float64).copy()
        self.target_write_reason = reason
        self.target_writer = writer

    def _open_debug_csv(self) -> None:
        if not self.debug_csv_path:
            return
        path = Path(self.debug_csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.debug_file = path.open("w", newline="", buffering=1)
        self.debug_writer = csv.writer(self.debug_file)
        self.debug_header = self._debug_csv_header()
        self.debug_writer.writerow(self.debug_header)

    def _debug_csv_header(self) -> list[str]:
        header = [
            "time",
            "state",
            "state_transition_reason",
            "deadman",
            "server_state",
            "first_target_x",
            "first_target_y",
            "first_target_z",
            "first_target_source",
            "first_target_to_internal_command_norm",
            "first_target_to_measured_norm",
            "internal_command_minus_measured_norm",
            "raw_x",
            "raw_y",
            "raw_z",
            "axis_sign_x",
            "axis_sign_y",
            "axis_sign_z",
            "input_after_axis_x",
            "input_after_axis_y",
            "input_after_axis_z",
            "translation_deadzone",
            "input_power",
            "u_deadzone_x",
            "u_deadzone_y",
            "u_deadzone_z",
            "u_scaled_x",
            "u_scaled_y",
            "u_scaled_z",
            "u_norm_before_clip",
            "u_norm_after_clip",
            "speed_scale",
            "fine_mode",
            "active_v_x_max",
            "active_v_y_max",
            "active_v_z_max",
            "active_a_x_max",
            "active_a_y_max",
            "active_a_z_max",
            "active_j_x_max",
            "active_j_y_max",
            "active_j_z_max",
            "d_move",
            "d_stop",
            "delta_x_max",
            "delta_y_max",
            "delta_z_max",
            "a_user_x",
            "a_user_y",
            "a_user_z",
            "a_des_x",
            "a_des_y",
            "a_des_z",
            "a_cmd_x",
            "a_cmd_y",
            "a_cmd_z",
            "v_cmd_x",
            "v_cmd_y",
            "v_cmd_z",
            "pre_velocity_limit_v_x",
            "pre_velocity_limit_v_y",
            "pre_velocity_limit_v_z",
            "post_velocity_limit_v_x",
            "post_velocity_limit_v_y",
            "post_velocity_limit_v_z",
            "delta_x",
            "delta_y",
            "delta_z",
            "accel_limited_x",
            "accel_limited_y",
            "accel_limited_z",
            "jerk_limited_x",
            "jerk_limited_y",
            "jerk_limited_z",
            "velocity_limited_x",
            "velocity_limited_y",
            "velocity_limited_z",
            "delta_limited_x",
            "delta_limited_y",
            "delta_limited_z",
            "steady_v_est_x",
            "steady_v_est_y",
            "steady_v_est_z",
            "server_target_x",
            "server_target_y",
            "server_target_z",
            "server_target_prev_x",
            "server_target_prev_y",
            "server_target_prev_z",
            "server_target_delta_x",
            "server_target_delta_y",
            "server_target_delta_z",
            "target_write_reason",
            "target_writer",
            "target_publish_count",
            "last_publish_reason",
            "publish_block_reason",
            "controller_raw_received_target_x",
            "controller_raw_received_target_y",
            "controller_raw_received_target_z",
            "controller_accepted_target_x",
            "controller_accepted_target_y",
            "controller_accepted_target_z",
            "controller_rt_target_x",
            "controller_rt_target_y",
            "controller_rt_target_z",
            "controller_internal_command_x",
            "controller_internal_command_y",
            "controller_internal_command_z",
            "controller_accept_targets",
            "target_accepted_count",
            "target_rejected_count",
            "last_target_reject_reason",
            "target_stream_primed",
            "has_target",
            "rt_has_target",
            "server_target_minus_measured_x",
            "server_target_minus_measured_y",
            "server_target_minus_measured_z",
            "server_target_minus_measured_norm",
            "server_target_minus_internal_command_x",
            "server_target_minus_internal_command_y",
            "server_target_minus_internal_command_z",
            "server_target_minus_internal_command_norm",
            "internal_command_minus_measured_x",
            "internal_command_minus_measured_y",
            "internal_command_minus_measured_z",
            "internal_command_minus_measured_norm",
            "accepted_target_minus_internal_command_x",
            "accepted_target_minus_internal_command_y",
            "accepted_target_minus_internal_command_z",
            "accepted_target_minus_internal_command_norm",
            "rt_target_minus_internal_command_x",
            "rt_target_minus_internal_command_y",
            "rt_target_minus_internal_command_z",
            "rt_target_minus_internal_command_norm",
            "target_v_x",
            "target_v_y",
            "target_v_z",
            "target_a_x",
            "target_a_y",
            "target_a_z",
            "target_j_x",
            "target_j_y",
            "target_j_z",
            "internal_command_v_x",
            "internal_command_v_y",
            "internal_command_v_z",
            "internal_command_a_x",
            "internal_command_a_y",
            "internal_command_a_z",
            "internal_command_j_x",
            "internal_command_j_y",
            "internal_command_j_z",
            "workspace_margin_low_x",
            "workspace_margin_low_y",
            "workspace_margin_low_z",
            "workspace_margin_high_x",
            "workspace_margin_high_y",
            "workspace_margin_high_z",
            "workspace_blocked_x",
            "workspace_blocked_y",
            "workspace_blocked_z",
            "workspace_slowdown_scale_x",
            "workspace_slowdown_scale_y",
            "workspace_slowdown_scale_z",
            "robot_state_fresh",
            "measured_pose_age_s",
            "measured_pose_update_count",
            "robot_state_callback_count",
            "ros2_controller_active",
            "controller_manager_available",
            "target_topic_publisher_count",
            "target_subscriber_count",
            "target_publish_rate_hz",
            "controller_update_period_s",
            "controller_update_overrun_count",
            "activation_command_to_measured_norm",
            "seeded_command_to_measured_norm",
            "command_seed_source",
            "measured_x",
            "measured_y",
            "measured_z",
            "command_pose_x",
            "command_pose_y",
            "command_pose_z",
            "published_target_x",
            "published_target_y",
            "published_target_z",
            "target_seed_source",
            "tracking_reference_source",
            "target_initialized_from_controller_command_pose",
            "control_target_valid_for_motion",
            "bottom_controller_targets_enabled",
            "controller_enable_targets_success",
            "final_delta_clamped",
            "target_jump_rejected_reason",
            "tracking_block_reason",
            "tracking_ref_type",
            "measured_pose_source",
            "measured_pose_topic",
            "measured_pose_timestamp",
            "measured_pose_header_age_s",
            "measured_pose_frame",
            "measured_pose_valid",
            "command_connected",
            "target_topic_publisher_count_ok",
            "last_publish_time",
            "last_publish_writer",
            "note",
        ]
        return header

    @staticmethod
    def _fmt_debug_value(value) -> str | int:
        if isinstance(value, (bool, np.bool_)):
            return int(value)
        if isinstance(value, (int, np.integer)):
            return int(value)
        if isinstance(value, (float, np.floating)):
            return f"{float(value):.12f}"
        return "" if value is None else str(value)

    @staticmethod
    def _vec3_or_nan(pose: np.ndarray | None) -> np.ndarray:
        if pose is None:
            return np.full(3, np.nan, dtype=np.float64)
        return np.asarray(pose[:3], dtype=np.float64)

    @staticmethod
    def _vec3_norm_delta(a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, float]:
        delta = a - b
        return delta, float(np.linalg.norm(delta))

    def _trace_derivatives(self, now: float, target: np.ndarray, internal: np.ndarray):
        nan = np.full(3, np.nan, dtype=np.float64)
        dt = now - self.prev_trace_time if self.prev_trace_time > 0.0 else math.nan
        if not math.isfinite(dt) or dt <= 1e-9:
            target_v = target_a = target_j = nan.copy()
            internal_v = internal_a = internal_j = nan.copy()
        else:
            target_v = (
                (target - self.prev_trace_target_position) / dt
                if self.prev_trace_target_position is not None
                else nan.copy()
            )
            target_a = (
                (target_v - self.prev_trace_target_velocity) / dt
                if self.prev_trace_target_velocity is not None and np.all(np.isfinite(target_v))
                else nan.copy()
            )
            target_j = (
                (target_a - self.prev_trace_target_acceleration) / dt
                if self.prev_trace_target_acceleration is not None and np.all(np.isfinite(target_a))
                else nan.copy()
            )
            internal_v = (
                (internal - self.prev_trace_internal_command_position) / dt
                if self.prev_trace_internal_command_position is not None
                else nan.copy()
            )
            internal_a = (
                (internal_v - self.prev_trace_internal_command_velocity) / dt
                if self.prev_trace_internal_command_velocity is not None and np.all(np.isfinite(internal_v))
                else nan.copy()
            )
            internal_j = (
                (internal_a - self.prev_trace_internal_command_acceleration) / dt
                if self.prev_trace_internal_command_acceleration is not None and np.all(np.isfinite(internal_a))
                else nan.copy()
            )
        if np.all(np.isfinite(target)):
            self.prev_trace_target_position = target.copy()
        if np.all(np.isfinite(target_v)):
            self.prev_trace_target_velocity = target_v.copy()
        if np.all(np.isfinite(target_a)):
            self.prev_trace_target_acceleration = target_a.copy()
        if np.all(np.isfinite(internal)):
            self.prev_trace_internal_command_position = internal.copy()
        if np.all(np.isfinite(internal_v)):
            self.prev_trace_internal_command_velocity = internal_v.copy()
        if np.all(np.isfinite(internal_a)):
            self.prev_trace_internal_command_acceleration = internal_a.copy()
        self.prev_trace_time = now
        return target_v, target_a, target_j, internal_v, internal_a, internal_j

    def _write_debug_row(self, now: float, note: str, step_result) -> None:
        if self.debug_writer is None:
            return

        config = self.motion_shaper.config
        measured = self._vec3_or_nan(self.measured_pose_from_franka)
        command_pose = self._vec3_or_nan(self.command_pose)
        first_target = self._vec3_or_nan(self.first_target_pose)
        server_target = self._vec3_or_nan(self.target_pose)
        previous_target = self._vec3_or_nan(
            self.previous_published_target_pose
            if self.previous_published_target_pose is not None
            else self.prev_trace_target_position
        )
        published = self._vec3_or_nan(self.published_target_pose)
        raw_received = self._vec3_or_nan(self.controller_raw_received_target_pose)
        accepted = self._vec3_or_nan(self.controller_accepted_target_pose)
        rt_target = self._vec3_or_nan(self.controller_rt_target_pose)
        internal_command = self._vec3_or_nan(self.controller_internal_command_pose)

        if step_result is None:
            input_after_deadzone = np.array(
                [
                    power_deadzone(v, config.translation_deadzone, 1.0)
                    for v in self.raw_action
                ],
                dtype=np.float64,
            )
            u_scaled = np.array(
                [
                    math.copysign(abs(v) ** max(1.0, config.input_power), v)
                    if v != 0.0
                    else 0.0
                    for v in input_after_deadzone
                ],
                dtype=np.float64,
            )
            u_norm_before_clip = float(np.linalg.norm(u_scaled))
            max_norm = max(1e-6, config.max_action_norm)
            if u_norm_before_clip > max_norm and u_norm_before_clip > 1e-12:
                u_scaled = u_scaled * (max_norm / u_norm_before_clip)
            u_norm_after_clip = float(np.linalg.norm(u_scaled))
            active_limits = self._read_axis_motion_limits("fine" if self.fine_mode else "coarse").scaled(config.speed_scale)
            z_reference = u_scaled[2] if abs(float(u_scaled[2])) > 1e-12 else self.motion_shaper.cmd_velocity[2]
            z_positive = bool(z_reference >= 0.0)
            active_v = np.array(
                [
                    active_limits.v_xy,
                    active_limits.v_xy,
                    active_limits.v_z_up if z_positive else active_limits.v_z_down,
                ],
                dtype=np.float64,
            )
            active_a = np.array(
                [
                    active_limits.a_xy,
                    active_limits.a_xy,
                    active_limits.a_z_up if z_positive else active_limits.a_z_down,
                ],
                dtype=np.float64,
            )
            active_j = np.array(
                [
                    active_limits.j_xy,
                    active_limits.j_xy,
                    active_limits.j_z_up if z_positive else active_limits.j_z_down,
                ],
                dtype=np.float64,
            )
            zeros = np.zeros(3, dtype=np.float64)
            nan = np.full(3, np.nan, dtype=np.float64)
            a_user = a_des = pre_v = post_v = delta = steady_v = zeros.copy()
            accel_limited = jerk_limited = velocity_limited = delta_limited = np.zeros(3, dtype=bool)
            workspace_blocked = np.zeros(3, dtype=bool)
            workspace_scale = np.ones(3, dtype=np.float64)
        else:
            input_after_deadzone = step_result.input_after_deadzone
            u_scaled = step_result.scaled_action
            u_norm_before_clip = step_result.u_norm_before_clip
            u_norm_after_clip = step_result.u_norm_after_clip
            active_v = step_result.active_velocity_limits
            active_a = step_result.active_acceleration_limits
            active_j = step_result.active_jerk_limits
            a_user = step_result.user_acceleration
            a_des = step_result.desired_acceleration
            pre_v = step_result.pre_velocity_limit_velocity
            post_v = step_result.post_velocity_limit_velocity
            delta = step_result.delta_position
            steady_v = step_result.steady_velocity_estimate
            accel_limited = step_result.accel_limited
            jerk_limited = step_result.jerk_limited
            velocity_limited = step_result.velocity_limited
            delta_limited = step_result.delta_limited
            workspace_blocked = step_result.workspace_clamped
            workspace_scale = step_result.workspace_slowdown_scale

        server_target_delta = server_target - previous_target
        target_minus_measured, target_minus_measured_norm = self._vec3_norm_delta(server_target, measured)
        target_minus_internal, target_minus_internal_norm = self._vec3_norm_delta(server_target, internal_command)
        internal_minus_measured, internal_minus_measured_norm = self._vec3_norm_delta(internal_command, measured)
        first_target_minus_internal, first_target_to_internal_norm = self._vec3_norm_delta(first_target, internal_command)
        first_target_minus_measured, first_target_to_measured_norm = self._vec3_norm_delta(first_target, measured)
        accepted_minus_internal, accepted_minus_internal_norm = self._vec3_norm_delta(accepted, internal_command)
        rt_minus_internal, rt_minus_internal_norm = self._vec3_norm_delta(rt_target, internal_command)
        target_v, target_a, target_j, internal_v, internal_a, internal_j = self._trace_derivatives(
            now, server_target, internal_command
        )
        workspace_margin_low = server_target - self.workspace_low
        workspace_margin_high = self.workspace_high - server_target
        measured_pose_age = self._measured_pose_age(now)
        command_pose_age = now - self.last_command_pose_time if self.last_command_pose_time > 0.0 else math.nan
        measured_header_age = (
            now - self.last_measured_pose_header_change_time
            if self.last_measured_pose_header_change_time > 0.0
            else math.nan
        )
        publish_rate = (
            1.0 / max(1e-9, self.last_target_publish_period_s)
            if math.isfinite(self.last_target_publish_period_s)
            and self.last_target_publish_period_s > 0.0
            else math.nan
        )
        target_subscriber_count = self.target_pub.get_subscription_count()
        controller_manager_available = self.controller_client.service_is_ready()
        row = {
            "time": now,
            "state": self.server_state.value,
            "state_transition_reason": self.state_transition_reason,
            "deadman": int(self.deadman_pressed),
            "server_state": self.server_state.value,
            "first_target_source": self.first_target_source,
            "first_target_to_internal_command_norm": first_target_to_internal_norm,
            "first_target_to_measured_norm": first_target_to_measured_norm,
            "internal_command_minus_measured_norm": internal_minus_measured_norm,
            "translation_deadzone": config.translation_deadzone,
            "input_power": config.input_power,
            "u_norm_before_clip": u_norm_before_clip,
            "u_norm_after_clip": u_norm_after_clip,
            "speed_scale": config.speed_scale,
            "fine_mode": int(self.fine_mode),
            "d_move": config.d_move,
            "d_stop": config.d_stop,
            "delta_x_max": config.delta_limits.xy,
            "delta_y_max": config.delta_limits.xy,
            "delta_z_max": config.delta_limits.z_up,
            "active_v_x_max": active_v[0],
            "active_v_y_max": active_v[1],
            "active_v_z_max": active_v[2],
            "active_a_x_max": active_a[0],
            "active_a_y_max": active_a[1],
            "active_a_z_max": active_a[2],
            "active_j_x_max": active_j[0],
            "active_j_y_max": active_j[1],
            "active_j_z_max": active_j[2],
            "target_write_reason": self.target_write_reason,
            "target_writer": self.target_writer,
            "target_publish_count": self.target_publish_count,
            "last_publish_reason": self.last_publish_reason,
            "publish_block_reason": self.publish_block_reason,
            "controller_accept_targets": self.controller_accept_targets,
            "target_accepted_count": self.controller_target_accepted_count,
            "target_rejected_count": self.controller_target_rejected_count,
            "last_target_reject_reason": self.controller_last_target_reject_reason,
            "target_stream_primed": self.controller_target_stream_primed,
            "has_target": self.controller_has_target,
            "rt_has_target": self.controller_rt_has_target,
            "server_target_minus_measured_norm": target_minus_measured_norm,
            "server_target_minus_internal_command_norm": target_minus_internal_norm,
            "internal_command_minus_measured_norm": internal_minus_measured_norm,
            "accepted_target_minus_internal_command_norm": accepted_minus_internal_norm,
            "rt_target_minus_internal_command_norm": rt_minus_internal_norm,
            "robot_state_fresh": int(self._robot_state_fresh(now)),
            "measured_pose_age_s": measured_pose_age,
            "measured_pose_update_count": self.measured_pose_update_count,
            "robot_state_callback_count": self.robot_state_callback_count,
            "ros2_controller_active": int(self.controller_active),
            "controller_manager_available": int(controller_manager_available),
            "target_topic_publisher_count": self.target_topic_publisher_count,
            "target_subscriber_count": target_subscriber_count,
            "target_publish_rate_hz": publish_rate,
            "controller_update_period_s": self.controller_update_period_s,
            "controller_update_overrun_count": self.controller_update_overrun_count,
            "activation_command_to_measured_norm": self.controller_activation_command_to_measured_norm,
            "seeded_command_to_measured_norm": self.controller_seeded_command_to_measured_norm,
            "command_seed_source": self.controller_command_seed_source,
            "target_seed_source": self.last_target_seed_source,
            "tracking_reference_source": self.last_tracking_reference_source,
            "target_initialized_from_controller_command_pose": int(self.target_initialized_from_controller_command_pose),
            "control_target_valid_for_motion": int(self.control_target_valid_for_motion),
            "bottom_controller_targets_enabled": int(self.bottom_controller_targets_enabled),
            "controller_enable_targets_success": int(self.last_controller_enable_targets_success),
            "final_delta_clamped": int(self.final_delta_clamped),
            "target_jump_rejected_reason": self.target_jump_rejected_reason,
            "tracking_block_reason": self.tracking_block_reason,
            "tracking_ref_type": self.tracking_ref_type,
            "measured_pose_source": self.measured_pose_source,
            "measured_pose_topic": self.measured_pose_topic,
            "measured_pose_timestamp": self.last_measured_pose_header_time,
            "measured_pose_header_age_s": measured_header_age,
            "measured_pose_frame": self.measured_pose_frame,
            "measured_pose_valid": int(self._measured_pose_valid()),
            "command_connected": int(self._command_connected()),
            "target_topic_publisher_count_ok": int(self.target_topic_publisher_count_ok),
            "last_publish_time": self.last_publish_time,
            "last_publish_writer": self.last_publish_writer,
            "note": note,
        }
        vector_fields = {
            "raw": self.raw_input_before_axis,
            "first_target": first_target,
            "axis_sign": self.axis_sign,
            "input_after_axis": self.raw_action,
            "u_deadzone": input_after_deadzone,
            "u_scaled": u_scaled,
            "a_user": a_user,
            "a_des": a_des,
            "a_cmd": self.motion_shaper.cmd_acceleration,
            "v_cmd": self.motion_shaper.cmd_velocity,
            "pre_velocity_limit_v": pre_v,
            "post_velocity_limit_v": post_v,
            "delta": delta,
            "accel_limited": accel_limited.astype(int),
            "jerk_limited": jerk_limited.astype(int),
            "velocity_limited": velocity_limited.astype(int),
            "delta_limited": delta_limited.astype(int),
            "steady_v_est": steady_v,
            "server_target": server_target,
            "server_target_prev": previous_target,
            "server_target_delta": server_target_delta,
            "controller_raw_received_target": raw_received,
            "controller_accepted_target": accepted,
            "controller_rt_target": rt_target,
            "controller_internal_command": internal_command,
            "server_target_minus_measured": target_minus_measured,
            "server_target_minus_internal_command": target_minus_internal,
            "internal_command_minus_measured": internal_minus_measured,
            "accepted_target_minus_internal_command": accepted_minus_internal,
            "rt_target_minus_internal_command": rt_minus_internal,
            "target_v": target_v,
            "target_a": target_a,
            "target_j": target_j,
            "internal_command_v": internal_v,
            "internal_command_a": internal_a,
            "internal_command_j": internal_j,
            "workspace_margin_low": workspace_margin_low,
            "workspace_margin_high": workspace_margin_high,
            "workspace_blocked": workspace_blocked.astype(int),
            "workspace_slowdown_scale": workspace_scale,
            "measured": measured,
            "command_pose": command_pose,
            "published_target": published,
        }
        for prefix, values in vector_fields.items():
            for axis, value in zip(("x", "y", "z"), values):
                row[f"{prefix}_{axis}"] = value
        self.debug_writer.writerow([self._fmt_debug_value(row.get(name, "")) for name in self.debug_header])
        self.debug_write_count += 1
        if self.debug_write_count % 1000 == 0 and self.debug_file is not None:
            self.debug_file.flush()

    def _update_controller_status(self, now: float) -> None:
        if not self.require_active_controller:
            return
        if self.pending_controller_future is not None:
            return
        if now - self.last_controller_check_time < self.controller_check_period_s:
            return
        self.last_controller_check_time = now
        if not self.controller_client.service_is_ready():
            self.controller_active = False
            self.controller_state = "controller_manager service unavailable"
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

    def _log_throttled(self, now: float, note: str) -> None:
        if now - self.last_log_time < 1.0 / max(0.1, self.log_rate_hz):
            return
        self.last_log_time = now
        self._publish_status(note)
        if self.measured_pose_from_franka is None or self.target_pose is None:
            self.get_logger().info(f"state={self.server_state.value} note={note}")
            return
        mode = "fine" if self.fine_mode else "coarse"
        debug_suffix = self._debug_suffix(time.monotonic())
        command_xyz = (
            self.command_pose[:3] if self.command_pose is not None else np.full(3, np.nan)
        )
        self.get_logger().info(
            "state=%s teleop=%s mode=%s controller=%s measured_xyz=%s command_xyz=%s target_xyz=%s published_xyz=%s target_minus_measured=%s v_cmd=%s a_cmd=%s action=%s block_reason=%s tracking_block=%s jump_rejected_reason=%s final_delta_clamped=%s write=%s/%s publish_count=%d last_publish=%s/%s publish_block=%s target_init_from_command=%s target_valid=%s bottom_targets=%s enable_success=%s target_seed_source=%s track_ref=%s measured_source=%s measured_topic=%s measured_frame=%s callbacks=%d reason=%s note=%s%s"
            % (
                self.server_state.value,
                self.teleop_state.value,
                mode,
                self.controller_state,
                np.array2string(
                    self.measured_pose_from_franka[:3], precision=4, suppress_small=True
                ),
                np.array2string(command_xyz, precision=4, suppress_small=True),
                np.array2string(self.target_pose[:3], precision=4, suppress_small=True),
                np.array2string(
                    self.published_target_pose[:3]
                    if self.published_target_pose is not None
                    else np.full(3, np.nan),
                    precision=4,
                    suppress_small=True,
                ),
                np.array2string(
                    self.target_pose[:3] - self.measured_pose_from_franka[:3],
                    precision=5,
                    suppress_small=True,
                ),
                np.array2string(self.motion_shaper.cmd_velocity, precision=5, suppress_small=True),
                np.array2string(
                    self.motion_shaper.cmd_acceleration, precision=5, suppress_small=True
                ),
                np.array2string(self.raw_action, precision=3, suppress_small=True),
                self.block_reason,
                self.tracking_block_reason,
                self.target_jump_rejected_reason,
                self.final_delta_clamped,
                self.target_write_reason,
                self.target_writer,
                self.target_publish_count,
                self.last_publish_reason,
                self.last_publish_writer,
                self.publish_block_reason,
                self.target_initialized_from_controller_command_pose,
                self.control_target_valid_for_motion,
                self.bottom_controller_targets_enabled,
                self.last_controller_enable_targets_success,
                self.last_target_seed_source,
                self.last_tracking_reference_source,
                self.measured_pose_source,
                self.measured_pose_topic,
                self.measured_pose_frame,
                self.robot_state_callback_count,
                self.state_transition_reason,
                note,
                debug_suffix,
            )
        )

    def _publish_status(self, note: str) -> None:
        now = time.monotonic()
        robot_state_fresh = self._robot_state_fresh(now)
        command_minus_measured = np.full(3, np.nan, dtype=np.float64)
        target_minus_command = np.full(3, np.nan, dtype=np.float64)
        first_target_minus_command = np.full(3, np.nan, dtype=np.float64)
        first_target_minus_measured = np.full(3, np.nan, dtype=np.float64)
        if self.controller_internal_command_pose is not None and self.measured_pose_from_franka is not None:
            command_minus_measured = (
                self.controller_internal_command_pose[:3] - self.measured_pose_from_franka[:3]
            )
        if self.target_pose is not None and self.controller_internal_command_pose is not None:
            target_minus_command = (
                self.target_pose[:3] - self.controller_internal_command_pose[:3]
            )
        if self.first_target_pose is not None and self.controller_internal_command_pose is not None:
            first_target_minus_command = (
                self.first_target_pose[:3] - self.controller_internal_command_pose[:3]
            )
        if self.first_target_pose is not None and self.measured_pose_from_franka is not None:
            first_target_minus_measured = (
                self.first_target_pose[:3] - self.measured_pose_from_franka[:3]
            )
        command_minus_measured_xy = float(np.linalg.norm(command_minus_measured[:2]))
        command_minus_measured_z = abs(float(command_minus_measured[2]))
        target_minus_command_xy = float(np.linalg.norm(target_minus_command[:2]))
        target_minus_command_z = abs(float(target_minus_command[2]))
        first_target_to_command_norm = float(np.linalg.norm(first_target_minus_command))
        first_target_to_measured_norm = float(np.linalg.norm(first_target_minus_measured))
        controller_internal_command_age = (
            now - self.last_controller_internal_command_pose_time
            if self.last_controller_internal_command_pose_time > 0.0
            else math.nan
        )
        message = (
            f"state={self.server_state.value} teleop={self.teleop_state.value} "
            f"controller={self.controller_state} "
            f"robot_state_fresh={robot_state_fresh} "
            f"measured_pose_source={self.measured_pose_source} "
            f"measured_pose_topic={self.measured_pose_topic} "
            f"measured_pose_frame={self.measured_pose_frame} "
            f"measured_pose_age_s={self._measured_pose_age(now):.6f} "
            f"measured_pose_header_age_s={self._measured_pose_header_age(now):.6f} "
            f"measured_pose_update_count={self.measured_pose_update_count} "
            f"robot_state_callback_count={self.robot_state_callback_count} "
            f"command_connected={self._command_connected()} "
            f"block_reason={self.block_reason} "
            f"tracking_block_reason={self.tracking_block_reason} "
            f"target_jump_rejected_reason={self.target_jump_rejected_reason} "
            f"write={self.target_write_reason}/{self.target_writer} "
            f"target_seed_source={self.last_target_seed_source} track_ref={self.last_tracking_reference_source} "
            f"first_target_source={self.first_target_source} "
            f"first_target_to_internal_command_norm={first_target_to_command_norm:.9f} "
            f"first_target_to_measured_norm={first_target_to_measured_norm:.9f} "
            f"controller_internal_command_pose_age_s={controller_internal_command_age:.6f} "
            f"command_minus_measured_xy={command_minus_measured_xy:.9f} "
            f"command_minus_measured_z={command_minus_measured_z:.9f} "
            f"target_minus_command_xy={target_minus_command_xy:.9f} "
            f"target_minus_command_z={target_minus_command_z:.9f} "
            f"tracking_start_tolerance={self.tracking_start_tolerance:.9f} "
            f"final_delta_clamped={self.final_delta_clamped} "
            f"target_publish_count={self.target_publish_count} "
            f"last_publish_reason={self.last_publish_reason} "
            f"last_publish_writer={self.last_publish_writer} "
            f"publish_block_reason={self.publish_block_reason} "
            f"target_initialized_from_controller_command_pose={self.target_initialized_from_controller_command_pose} "
            f"control_target_valid_for_motion={self.control_target_valid_for_motion} "
            f"bottom_controller_targets_enabled={self.bottom_controller_targets_enabled} "
            f"controller_enable_targets_success={self.last_controller_enable_targets_success} "
            f"tracking_ref_type={self.tracking_ref_type} "
            f"reason={self.state_transition_reason} note={note}"
        )
        if message == self.last_status_message:
            return
        self.last_status_message = message
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def _debug_suffix(self, now: float) -> str:
        ages = (
            f" measured_pose_age={now - self.last_pose_time:.4f}"
            if self.last_pose_time > 0.0
            else " measured_pose_age=nan"
        )
        ages += (
            f" command_pose_age={now - self.last_command_pose_time:.4f}"
            if self.last_command_pose_time > 0.0
            else " command_pose_age=nan"
        )
        ages += (
            f" measured_pose_frame={self.measured_pose_frame}"
            f" measured_pose_callbacks={self.robot_state_callback_count}"
            f" measured_pose_header_age={self._measured_pose_header_age(now):.4f}"
            f" command_connected={self._command_connected()}"
        )
        if self.last_step_result is None:
            return (
                f" robot_state_fresh={self._robot_state_fresh(now)}"
                f" command_pose_fresh={self._command_pose_fresh(now)}{ages}"
            )
        result = self.last_step_result
        return (
            " pre_guard_target=%s post_guard_target=%s"
            " pre_guard_v_cmd=%s post_guard_v_cmd=%s"
            " workspace_clamped=%s workspace_slowdown_scale=%s"
            " tracking_error_guard_triggered=%s robot_state_fresh=%s command_pose_fresh=%s%s"
            % (
                np.array2string(result.pre_guard_target, precision=5, suppress_small=True),
                np.array2string(result.post_guard_target, precision=5, suppress_small=True),
                np.array2string(result.pre_guard_velocity, precision=5, suppress_small=True),
                np.array2string(result.post_guard_velocity, precision=5, suppress_small=True),
                np.array2string(result.workspace_clamped.astype(int), separator=","),
                np.array2string(result.workspace_slowdown_scale, precision=3, suppress_small=True),
                self.last_tracking_error_guard_triggered,
                self._robot_state_fresh(now),
                self._command_pose_fresh(now),
                ages,
            )
        )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PoseActionServerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
