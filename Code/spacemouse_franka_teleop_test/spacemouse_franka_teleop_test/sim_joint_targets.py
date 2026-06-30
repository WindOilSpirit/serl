#!/usr/bin/env python3
"""Offline 10 s SpaceMouse/Franka shaping simulation with IK joint target output."""

from __future__ import annotations

import argparse
import csv
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from .motion_shaping import (
    MotionShaper,
    build_sim_motion_shaper,
)


DEFAULT_Q = np.array(
    [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483],
    dtype=np.float64,
)
DEFAULT_URDF = Path("/home/admin123/ros2_ws/src/franka_hardware/test/fr3.urdf")
DEFAULT_OUTPUT = Path(
    "/home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test/sim_outputs/joint_targets_10s.csv"
)

SIM_EXTRA_HEADER = [
    "axis_sign_x",
    "axis_sign_y",
    "axis_sign_z",
    "input_after_axis_x",
    "input_after_axis_y",
    "input_after_axis_z",
    "first_target_x",
    "first_target_y",
    "first_target_z",
    "first_target_source",
    "first_target_to_internal_command_norm",
    "first_target_to_measured_norm",
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
    "target_topic_publisher_count",
    "target_subscriber_count",
    "target_publish_rate_hz",
    "controller_update_period_s",
    "controller_update_overrun_count",
    "activation_command_to_measured_norm",
    "seeded_command_to_measured_norm",
    "command_seed_source",
]


@dataclass
class SpaceMouseSample:
    action: np.ndarray
    buttons: list[int]
    timestamp: float


class SpaceMouseReader:
    def __init__(self) -> None:
        import pyspacemouse  # type: ignore

        self._lock = threading.Lock()
        self._sample = SpaceMouseSample(np.zeros(6, dtype=np.float64), [0, 0], time.monotonic())
        self._stop = threading.Event()
        self._device = pyspacemouse.open()
        if self._device is None:
            raise RuntimeError("pyspacemouse.open() failed")
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self._device is not None:
            self._device.close()

    def latest(self) -> SpaceMouseSample:
        with self._lock:
            return SpaceMouseSample(
                self._sample.action.copy(),
                list(self._sample.buttons),
                self._sample.timestamp,
            )

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            state = self._device.read()
            if state is None:
                time.sleep(0.001)
                continue
            action = np.array(
                [-state.y, state.x, state.z, -state.roll, -state.pitch, -state.yaw],
                dtype=np.float64,
            )
            with self._lock:
                self._sample = SpaceMouseSample(action, list(state.buttons), time.monotonic())


def make_default_shaper() -> MotionShaper:
    return build_sim_motion_shaper()


def load_model(urdf_path: Path) -> tuple[pin.Model, pin.Data, int]:
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    frame_id = model.getFrameId("fr3_link8")
    if frame_id == len(model.frames):
        raise RuntimeError("fr3_link8 frame not found in URDF")
    return model, data, frame_id


def forward_pose(model: pin.Model, data: pin.Data, frame_id: int, q: np.ndarray) -> pin.SE3:
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[frame_id].copy()


def solve_position_ik(
    model: pin.Model,
    data: pin.Data,
    frame_id: int,
    q_seed: np.ndarray,
    target_pose: pin.SE3,
    max_iters: int = 20,
) -> np.ndarray:
    q = q_seed.copy()
    for _ in range(max_iters):
        current_pose = forward_pose(model, data, frame_id, q)
        error = pin.log6(current_pose.actInv(target_pose)).vector
        if float(np.linalg.norm(error)) < 1e-10:
            break
        jacobian = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)
        damping = 1e-8
        dq = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(6), error)
        q = pin.integrate(model, q, 0.5 * dq)
        q = np.minimum(np.maximum(q, model.lowerPositionLimit), model.upperPositionLimit)
    return q


def run_simulation(args: argparse.Namespace) -> dict[str, float]:
    model, data, frame_id = load_model(args.urdf)
    shaper = make_default_shaper()
    workspace_low = np.array(args.workspace_low, dtype=np.float64)
    workspace_high = np.array(args.workspace_high, dtype=np.float64)

    q = DEFAULT_Q.copy()
    current_pose = forward_pose(model, data, frame_id, q)
    target_pose = current_pose.copy()
    first_target_translation = target_pose.translation.copy()
    initial_q = q.copy()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fixed_action = np.array(args.action, dtype=np.float64)
    spacemouse = None
    if not args.no_spacemouse:
        spacemouse = SpaceMouseReader()

    max_joint_delta = 0.0
    max_cartesian_delta = 0.0
    max_velocity_norm = 0.0
    max_acceleration_norm = 0.0
    active_frames = 0
    nonzero_input_frames = 0

    try:
        with args.output.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "frame",
                    "time_s",
                    "state_transition_reason",
                    "deadman",
                    "fine",
                    "raw_x",
                    "raw_y",
                    "raw_z",
                    "u_scaled_z",
                    "a_user_z",
                    "a_des_z",
                    "velocity_saturation_scale_x",
                    "velocity_saturation_scale_y",
                    "velocity_saturation_scale_z",
                    "q1",
                    "q2",
                    "q3",
                    "q4",
                    "q5",
                    "q6",
                    "q7",
                    "measured_x",
                    "measured_y",
                    "measured_z",
                    "target_x",
                    "target_y",
                    "target_z",
                    "target_minus_measured_x",
                    "target_minus_measured_y",
                    "target_minus_measured_z",
                    "v_cmd_x",
                    "v_cmd_y",
                    "v_cmd_z",
                    "a_cmd_x",
                    "a_cmd_y",
                    "a_cmd_z",
                    "pre_guard_target_x",
                    "pre_guard_target_y",
                    "pre_guard_target_z",
                    "post_guard_target_x",
                    "post_guard_target_y",
                    "post_guard_target_z",
                    "pre_guard_v_cmd_x",
                    "pre_guard_v_cmd_y",
                    "pre_guard_v_cmd_z",
                    "post_guard_v_cmd_x",
                    "post_guard_v_cmd_y",
                    "post_guard_v_cmd_z",
                    "tracking_error_guard_triggered",
                    "block_reason",
                    "tracking_block_reason",
                    "target_jump_rejected_reason",
                    "workspace_clamped_x",
                    "workspace_clamped_y",
                    "workspace_clamped_z",
                    "workspace_slowdown_scale_x",
                    "workspace_slowdown_scale_y",
                    "workspace_slowdown_scale_z",
                    "z_at_upper_bound",
                    "z_at_lower_bound",
                    "pre_velocity_limit_v_cmd_z",
                    "post_velocity_limit_v_cmd_z",
                    "velocity_limited_z",
                    "controller_active",
                    "robot_state_fresh",
                    "state",
                    *SIM_EXTRA_HEADER,
                ]
            )
            start_time = time.monotonic()
            prev_target_trace = target_pose.translation.copy()
            prev_target_velocity = np.full(3, np.nan, dtype=np.float64)
            prev_target_acceleration = np.full(3, np.nan, dtype=np.float64)
            for frame in range(args.frames):
                t = frame * args.dt
                sample = (
                    spacemouse.latest()
                    if spacemouse is not None
                    else SpaceMouseSample(fixed_action.copy(), [int(args.deadman), 0], time.monotonic())
                )
                if args.scenario == "z_oscillation":
                    action, deadman_active, fine_mode = z_oscillation_sample(t)
                    deadman_active = deadman_active or args.force_deadman
                else:
                    action = sample.action[:3].copy() if spacemouse is not None else fixed_action.copy()
                    deadman_active = args.force_deadman or _button(sample.buttons, args.deadman_button_index)
                    fine_mode = _button(sample.buttons, args.fine_button_index)
                state = "HUMAN_CONTROL" if deadman_active else "BRAKE"
                if not deadman_active and shaper.is_stopped():
                    state = "IDLE"
                state_transition_reason = "deadman active" if deadman_active else "deadman released"
                active_frames += int(deadman_active)
                nonzero_input_frames += int(np.any(np.abs(action) > 1e-9))
                target_before_step = target_pose.translation.copy()

                result = shaper.step(
                    action,
                    deadman_active,
                    fine_mode,
                    args.dt,
                    target_pose.translation,
                    workspace_low,
                    workspace_high,
                )
                target_pose.translation = target_pose.translation + result.delta_position
                target_velocity = (
                    (target_pose.translation - prev_target_trace) / args.dt
                    if frame > 0
                    else np.full(3, np.nan, dtype=np.float64)
                )
                target_acceleration = (
                    (target_velocity - prev_target_velocity) / args.dt
                    if frame > 1 and np.all(np.isfinite(target_velocity))
                    else np.full(3, np.nan, dtype=np.float64)
                )
                target_jerk = (
                    (target_acceleration - prev_target_acceleration) / args.dt
                    if frame > 2 and np.all(np.isfinite(target_acceleration))
                    else np.full(3, np.nan, dtype=np.float64)
                )
                prev_target_trace = target_pose.translation.copy()
                if np.all(np.isfinite(target_velocity)):
                    prev_target_velocity = target_velocity.copy()
                if np.all(np.isfinite(target_acceleration)):
                    prev_target_acceleration = target_acceleration.copy()
                q = solve_position_ik(model, data, frame_id, q, target_pose, args.ik_iters)
                measured_pose = forward_pose(model, data, frame_id, q)
                target_minus_measured = target_pose.translation - measured_pose.translation
                workspace_margin_low = target_pose.translation - workspace_low
                workspace_margin_high = workspace_high - target_pose.translation
                nan3 = np.full(3, np.nan, dtype=np.float64)

                joint_delta = float(np.max(np.abs(q - initial_q)))
                cartesian_delta = float(np.linalg.norm(target_pose.translation - current_pose.translation))
                max_joint_delta = max(max_joint_delta, joint_delta)
                max_cartesian_delta = max(max_cartesian_delta, cartesian_delta)
                max_velocity_norm = max(max_velocity_norm, float(np.linalg.norm(shaper.cmd_velocity)))
                max_acceleration_norm = max(
                    max_acceleration_norm, float(np.linalg.norm(shaper.cmd_acceleration))
                )

                writer.writerow(
                    [
                        frame,
                        f"{frame * args.dt:.6f}",
                        state_transition_reason,
                        int(deadman_active),
                        int(fine_mode),
                        *[f"{value:.12f}" for value in action],
                        f"{result.scaled_action[2]:.12f}",
                        f"{result.user_acceleration[2]:.12f}",
                        f"{result.desired_acceleration[2]:.12f}",
                        *[f"{value:.12f}" for value in result.velocity_saturation_scale],
                        *[f"{value:.12f}" for value in q],
                        *[f"{value:.12f}" for value in measured_pose.translation],
                        *[f"{value:.12f}" for value in target_pose.translation],
                        *[f"{value:.12f}" for value in target_minus_measured],
                        *[f"{value:.12f}" for value in shaper.cmd_velocity],
                        *[f"{value:.12f}" for value in shaper.cmd_acceleration],
                        *[f"{value:.12f}" for value in result.pre_guard_target],
                        *[f"{value:.12f}" for value in result.post_guard_target],
                        *[f"{value:.12f}" for value in result.pre_guard_velocity],
                        *[f"{value:.12f}" for value in result.post_guard_velocity],
                        0,
                        "",
                        "",
                        "",
                        *[int(value) for value in result.workspace_clamped],
                        *[f"{value:.12f}" for value in result.workspace_slowdown_scale],
                        int(result.z_at_upper_bound),
                        int(result.z_at_lower_bound),
                        f"{result.pre_velocity_limit_velocity[2]:.12f}",
                        f"{result.post_velocity_limit_velocity[2]:.12f}",
                        int(result.velocity_limited[2]),
                        1,
                        1,
                        state,
                        "1.000000000000",
                        "1.000000000000",
                        "1.000000000000",
                        *[f"{value:.12f}" for value in action],
                        *[f"{value:.12f}" for value in first_target_translation],
                        "offline_sim_initial_target",
                        "nan",
                        "0.000000000000",
                        f"{shaper.config.translation_deadzone:.12f}",
                        f"{shaper.config.input_power:.12f}",
                        *[f"{value:.12f}" for value in result.input_after_deadzone],
                        *[f"{value:.12f}" for value in result.scaled_action],
                        f"{result.u_norm_before_clip:.12f}",
                        f"{result.u_norm_after_clip:.12f}",
                        f"{shaper.config.speed_scale:.12f}",
                        int(fine_mode),
                        *[f"{value:.12f}" for value in result.active_velocity_limits],
                        *[f"{value:.12f}" for value in result.active_acceleration_limits],
                        *[f"{value:.12f}" for value in result.active_jerk_limits],
                        f"{shaper.config.d_move:.12f}",
                        f"{shaper.config.d_stop:.12f}",
                        f"{shaper.config.delta_limits.xy:.12f}",
                        f"{shaper.config.delta_limits.xy:.12f}",
                        f"{shaper.config.delta_limits.z_up:.12f}",
                        *[f"{value:.12f}" for value in result.user_acceleration],
                        *[f"{value:.12f}" for value in result.desired_acceleration],
                        *[f"{value:.12f}" for value in result.pre_velocity_limit_velocity],
                        *[f"{value:.12f}" for value in result.post_velocity_limit_velocity],
                        *[f"{value:.12f}" for value in result.delta_position],
                        *[int(value) for value in result.accel_limited],
                        *[int(value) for value in result.jerk_limited],
                        *[int(value) for value in result.velocity_limited],
                        *[int(value) for value in result.delta_limited],
                        *[f"{value:.12f}" for value in result.steady_velocity_estimate],
                        *[f"{value:.12f}" for value in target_pose.translation],
                        *[f"{value:.12f}" for value in target_before_step],
                        *[f"{value:.12f}" for value in result.delta_position],
                        "SIM_INTEGRATION",
                        "sim_joint_targets",
                        frame + 1,
                        "SIM_INTEGRATION",
                        "",
                        *["nan" for _ in range(9)],
                        *["nan" for _ in range(3)],
                        "False",
                        0,
                        0,
                        "offline_sim_no_bottom_controller",
                        "False",
                        "False",
                        "False",
                        *[f"{value:.12f}" for value in target_minus_measured],
                        f"{float(np.linalg.norm(target_minus_measured)):.12f}",
                        *["nan" for _ in range(16)],
                        *[f"{value:.12f}" if math.isfinite(float(value)) else "nan" for value in target_velocity],
                        *[f"{value:.12f}" if math.isfinite(float(value)) else "nan" for value in target_acceleration],
                        *[f"{value:.12f}" if math.isfinite(float(value)) else "nan" for value in target_jerk],
                        *["nan" for _ in range(9)],
                        *[f"{value:.12f}" for value in workspace_margin_low],
                        *[f"{value:.12f}" for value in workspace_margin_high],
                        *[int(value) for value in result.workspace_clamped],
                        0,
                        0,
                        f"{1.0 / args.dt:.12f}",
                        "nan",
                        0,
                        "nan",
                        "nan",
                        "offline_sim_no_controller_activation",
                    ]
                )

                if args.realtime:
                    next_time = start_time + (frame + 1) * args.dt
                    sleep_s = next_time - time.monotonic()
                    if sleep_s > 0.0:
                        time.sleep(sleep_s)
    finally:
        if spacemouse is not None:
            spacemouse.close()

    return {
        "frames": float(args.frames),
        "duration_s": args.frames * args.dt,
        "active_frames": float(active_frames),
        "nonzero_input_frames": float(nonzero_input_frames),
        "max_joint_delta_rad": max_joint_delta,
        "max_cartesian_delta_m": max_cartesian_delta,
        "max_velocity_norm_mps": max_velocity_norm,
        "max_acceleration_norm_mps2": max_acceleration_norm,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--frames", type=int, default=10000)
    parser.add_argument("--dt", type=float, default=0.001)
    parser.add_argument("--ik-iters", type=int, default=20)
    parser.add_argument("--realtime", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--no-spacemouse", action="store_true")
    parser.add_argument("--workspace-low", type=float, nargs=3, default=[0.25, -0.20, 0.04])
    parser.add_argument("--workspace-high", type=float, nargs=3, default=[0.75, 0.25, 0.75])
    parser.add_argument(
        "--scenario",
        choices=["fixed", "z_oscillation"],
        default="fixed",
        help="Use z_oscillation for a synthetic deadman/up-down test without SpaceMouse.",
    )
    parser.add_argument("--deadman", action="store_true")
    parser.add_argument("--force-deadman", action="store_true")
    parser.add_argument("--deadman-button-index", type=int, default=0)
    parser.add_argument("--fine-button-index", type=int, default=1)
    parser.add_argument("--action", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    return parser.parse_args()


def _button(buttons: list[int], index: int) -> bool:
    return 0 <= index < len(buttons) and bool(buttons[index])


def z_oscillation_sample(t: float) -> tuple[np.ndarray, bool, bool]:
    if t < 2.8:
        return np.zeros(3, dtype=np.float64), False, False

    phase_t = t - 2.8
    half_period = 0.8
    sign = -1.0 if int(phase_t / half_period) % 2 == 0 else 1.0
    return np.array([0.0, 0.0, 0.35 * sign], dtype=np.float64), True, False


def main() -> None:
    args = parse_args()
    summary = run_simulation(args)
    print(f"wrote {args.output}")
    for key, value in summary.items():
        print(f"{key}: {value:.12g}")


if __name__ == "__main__":
    main()
