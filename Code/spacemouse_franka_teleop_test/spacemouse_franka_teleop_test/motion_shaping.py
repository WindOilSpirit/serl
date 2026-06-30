#!/usr/bin/env python3
"""Velocity-based target generation for SpaceMouse Cartesian teleoperation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass
class AxisMotionLimits:
    v_xy: float
    v_z_up: float
    v_z_down: float
    a_xy: float
    a_z_up: float
    a_z_down: float
    j_xy: float
    j_z_up: float
    j_z_down: float

    def scaled(self, scale: float) -> "AxisMotionLimits":
        scale = max(0.0, float(scale))
        return AxisMotionLimits(
            v_xy=max(0.0, self.v_xy) * scale,
            v_z_up=max(0.0, self.v_z_up) * scale,
            v_z_down=max(0.0, self.v_z_down) * scale,
            a_xy=max(0.0, self.a_xy) * scale,
            a_z_up=max(0.0, self.a_z_up) * scale,
            a_z_down=max(0.0, self.a_z_down) * scale,
            j_xy=max(0.0, self.j_xy) * scale,
            j_z_up=max(0.0, self.j_z_up) * scale,
            j_z_down=max(0.0, self.j_z_down) * scale,
        )


@dataclass
class StepDeltaLimits:
    xy: float
    z_up: float
    z_down: float


@dataclass
class TrackingErrorLimits:
    xy: float
    z: float


@dataclass
class MotionShapingConfig:
    speed_scale: float
    translation_deadzone: float
    rotation_deadzone: float
    input_power: float
    max_action_norm: float
    coarse_limits: AxisMotionLimits
    fine_limits: AxisMotionLimits
    d_move: float
    d_stop: float
    dt_nominal: float
    dt_max: float
    delta_limits: StepDeltaLimits
    workspace_slowdown_distance: float


@dataclass
class MotionStepResult:
    delta_position: np.ndarray
    dt_used: float
    braking: bool
    stale_dt: bool
    workspace_clamped: np.ndarray
    workspace_slowdown_scale: np.ndarray
    z_at_upper_bound: bool
    z_at_lower_bound: bool
    pre_guard_velocity: np.ndarray
    post_guard_velocity: np.ndarray
    pre_guard_target: np.ndarray
    post_guard_target: np.ndarray
    scaled_action: np.ndarray
    input_after_deadzone: np.ndarray
    u_norm_before_clip: float
    u_norm_after_clip: float
    active_velocity_limits: np.ndarray
    active_acceleration_limits: np.ndarray
    active_jerk_limits: np.ndarray
    user_acceleration: np.ndarray
    desired_acceleration: np.ndarray
    steady_velocity_estimate: np.ndarray
    velocity_saturation_scale: np.ndarray
    pre_velocity_limit_velocity: np.ndarray
    post_velocity_limit_velocity: np.ndarray
    accel_limited: np.ndarray
    jerk_limited: np.ndarray
    velocity_limited: np.ndarray
    delta_limited: np.ndarray
    user_velocity: np.ndarray
    desired_velocity: np.ndarray


TARGET_GENERATION_PARAMETER_DEFAULTS: dict[str, object] = {
    "speed_scale": 1.0,
    "spacemouse_target_scale": 1.0,
    "normal_spacemouse_target_scale": 2.0,
    "fine_spacemouse_target_scale": 1.0,
    "translation_deadzone": 0.06,
    "rotation_deadzone": 0.10,
    "input_power": 3.0,
    "max_action_norm": 1.0,
    "coarse_v_xy_max": 0.003,
    "coarse_v_z_up_max": 0.003,
    "coarse_v_z_down_max": 0.001,
    "coarse_a_xy_max": 0.010,
    "coarse_a_z_up_max": 0.010,
    "coarse_a_z_down_max": 0.003,
    "coarse_j_xy_max": 0.100,
    "coarse_j_z_up_max": 0.100,
    "coarse_j_z_down_max": 0.030,
    "fine_v_xy_max": 0.001,
    "fine_v_z_up_max": 0.001,
    "fine_v_z_down_max": 0.0005,
    "fine_a_xy_max": 0.004,
    "fine_a_z_up_max": 0.004,
    "fine_a_z_down_max": 0.0015,
    "fine_j_xy_max": 0.040,
    "fine_j_z_up_max": 0.040,
    "fine_j_z_down_max": 0.015,
    "d_move": 3.0,
    "d_stop": 5.0,
    "dt_nominal": 0.004,
    "dt_max": 0.020,
    "delta_xy_max": 0.000020,
    "delta_z_up_max": 0.000020,
    "delta_z_down_max": 0.000010,
    "workspace_slowdown_distance": 0.030,
    "workspace_low": [0.25, -0.20, 0.04],
    "workspace_high": [0.75, 0.25, 0.75],
}

SIM_MOTION_SHAPING_DEFAULTS: dict[str, object] = {
    "speed_scale": 1.0,
    "translation_deadzone": 0.04,
    "rotation_deadzone": 0.10,
    "input_power": 2.0,
    "max_action_norm": 1.0,
    "coarse_limits": AxisMotionLimits(
        v_xy=0.005,
        v_z_up=0.003,
        v_z_down=0.001,
        a_xy=0.008,
        a_z_up=0.006,
        a_z_down=0.002,
        j_xy=0.050,
        j_z_up=0.030,
        j_z_down=0.020,
    ),
    "fine_limits": AxisMotionLimits(
        v_xy=0.0015,
        v_z_up=0.001,
        v_z_down=0.0005,
        a_xy=0.003,
        a_z_up=0.002,
        a_z_down=0.001,
        j_xy=0.020,
        j_z_up=0.015,
        j_z_down=0.010,
    ),
    "d_move": 1.5,
    "d_stop": 3.0,
    "dt_nominal": 0.001,
    "dt_max": 0.003,
    "delta_limits": StepDeltaLimits(xy=0.000010, z_up=0.000010, z_down=0.000003),
    "workspace_slowdown_distance": 0.030,
}


def deadzone_normalized(value: float, deadzone: float) -> float:
    value = float(np.clip(value, -1.0, 1.0))
    if abs(value) <= deadzone:
        return 0.0
    normalized = (abs(value) - deadzone) / max(1e-9, 1.0 - deadzone)
    return math.copysign(normalized, value)


def power_deadzone(value: float, deadzone: float, power: float) -> float:
    normalized = deadzone_normalized(value, deadzone)
    if normalized == 0.0:
        return 0.0
    shaped = normalized ** max(1.0, float(power))
    return math.copysign(abs(shaped), normalized)


def declare_target_generation_parameters(node: "Node") -> None:
    for name, default in TARGET_GENERATION_PARAMETER_DEFAULTS.items():
        if isinstance(default, list):
            node.declare_parameter(name, list(default))
        else:
            node.declare_parameter(name, default)


def get_target_generation_config(node: "Node") -> MotionShapingConfig:
    def limits(prefix: str) -> AxisMotionLimits:
        return AxisMotionLimits(
            v_xy=float(node.get_parameter(f"{prefix}_v_xy_max").value),
            v_z_up=float(node.get_parameter(f"{prefix}_v_z_up_max").value),
            v_z_down=float(node.get_parameter(f"{prefix}_v_z_down_max").value),
            a_xy=float(node.get_parameter(f"{prefix}_a_xy_max").value),
            a_z_up=float(node.get_parameter(f"{prefix}_a_z_up_max").value),
            a_z_down=float(node.get_parameter(f"{prefix}_a_z_down_max").value),
            j_xy=float(node.get_parameter(f"{prefix}_j_xy_max").value),
            j_z_up=float(node.get_parameter(f"{prefix}_j_z_up_max").value),
            j_z_down=float(node.get_parameter(f"{prefix}_j_z_down_max").value),
        )

    return MotionShapingConfig(
        speed_scale=effective_target_generation_scale(node, False),
        translation_deadzone=float(node.get_parameter("translation_deadzone").value),
        rotation_deadzone=float(node.get_parameter("rotation_deadzone").value),
        input_power=float(node.get_parameter("input_power").value),
        max_action_norm=float(node.get_parameter("max_action_norm").value),
        coarse_limits=limits("coarse"),
        fine_limits=limits("fine"),
        d_move=float(node.get_parameter("d_move").value),
        d_stop=float(node.get_parameter("d_stop").value),
        dt_nominal=float(node.get_parameter("dt_nominal").value),
        dt_max=float(node.get_parameter("dt_max").value),
        delta_limits=StepDeltaLimits(
            xy=float(node.get_parameter("delta_xy_max").value),
            z_up=float(node.get_parameter("delta_z_up_max").value),
            z_down=float(node.get_parameter("delta_z_down_max").value),
        ),
        workspace_slowdown_distance=float(node.get_parameter("workspace_slowdown_distance").value),
    )


def build_motion_shaper(node: "Node") -> "MotionShaper":
    return MotionShaper(get_target_generation_config(node))


def effective_target_generation_scale(node: "Node", fine_mode: bool) -> float:
    speed_scale = float(node.get_parameter("speed_scale").value)
    global_scale = float(node.get_parameter("spacemouse_target_scale").value)
    mode_scale_name = (
        "fine_spacemouse_target_scale" if fine_mode else "normal_spacemouse_target_scale"
    )
    mode_scale = float(node.get_parameter(mode_scale_name).value)
    return max(0.0, speed_scale * global_scale * mode_scale)


def refresh_motion_shaper_scale(node: "Node", motion_shaper: "MotionShaper", fine_mode: bool) -> None:
    motion_shaper.config.speed_scale = effective_target_generation_scale(node, fine_mode)


def get_workspace_bounds(node: "Node") -> tuple[np.ndarray, np.ndarray]:
    low = np.asarray(node.get_parameter("workspace_low").value, dtype=np.float64)
    high = np.asarray(node.get_parameter("workspace_high").value, dtype=np.float64)
    return low, high


def build_sim_motion_shaper() -> "MotionShaper":
    return MotionShaper(
        MotionShapingConfig(
            speed_scale=float(SIM_MOTION_SHAPING_DEFAULTS["speed_scale"]),
            translation_deadzone=float(SIM_MOTION_SHAPING_DEFAULTS["translation_deadzone"]),
            rotation_deadzone=float(SIM_MOTION_SHAPING_DEFAULTS["rotation_deadzone"]),
            input_power=float(SIM_MOTION_SHAPING_DEFAULTS["input_power"]),
            max_action_norm=float(SIM_MOTION_SHAPING_DEFAULTS["max_action_norm"]),
            coarse_limits=SIM_MOTION_SHAPING_DEFAULTS["coarse_limits"],
            fine_limits=SIM_MOTION_SHAPING_DEFAULTS["fine_limits"],
            d_move=float(SIM_MOTION_SHAPING_DEFAULTS["d_move"]),
            d_stop=float(SIM_MOTION_SHAPING_DEFAULTS["d_stop"]),
            dt_nominal=float(SIM_MOTION_SHAPING_DEFAULTS["dt_nominal"]),
            dt_max=float(SIM_MOTION_SHAPING_DEFAULTS["dt_max"]),
            delta_limits=SIM_MOTION_SHAPING_DEFAULTS["delta_limits"],
            workspace_slowdown_distance=float(
                SIM_MOTION_SHAPING_DEFAULTS["workspace_slowdown_distance"]
            ),
        )
    )


class MotionShaper:
    """Convert normalized SpaceMouse intent into target-position deltas."""

    def __init__(self, config: MotionShapingConfig) -> None:
        self.config = config
        self.cmd_velocity = np.zeros(3, dtype=np.float64)
        self.cmd_acceleration = np.zeros(3, dtype=np.float64)

    def reset(self) -> None:
        self.cmd_velocity[:] = 0.0
        self.cmd_acceleration[:] = 0.0

    def is_stopped(self) -> bool:
        return (
            float(np.linalg.norm(self.cmd_velocity)) < 1e-6
            and float(np.linalg.norm(self.cmd_acceleration)) < 1e-5
        )

    def translation_active(self, action: np.ndarray) -> bool:
        return any(abs(float(v)) > self.config.translation_deadzone for v in action)

    def shape_rotation(self, rotation: np.ndarray) -> np.ndarray:
        return np.array(
            [
                power_deadzone(v, self.config.rotation_deadzone, self.config.input_power)
                for v in rotation
            ],
            dtype=np.float64,
        )

    def step(
        self,
        action: np.ndarray,
        deadman_active: bool,
        fine_mode: bool,
        dt: float,
        target_position: np.ndarray,
        workspace_low: np.ndarray,
        workspace_high: np.ndarray,
    ) -> MotionStepResult:
        dt_used, stale_dt = self._effective_dt(dt)
        limits = self._mode_limits(fine_mode)
        input_after_deadzone, scaled_action, u_norm_before_clip, u_norm_after_clip = (
            self._scaled_action_with_debug(action)
        )
        z_direction_reference = (
            scaled_action[2] if abs(float(scaled_action[2])) > 1e-12 else self.cmd_velocity[2]
        )
        z_positive_direction = bool(z_direction_reference >= 0.0)
        active_velocity_limits = np.array(
            [
                limits.v_xy,
                limits.v_xy,
                limits.v_z_up if z_positive_direction else limits.v_z_down,
            ],
            dtype=np.float64,
        )
        active_acceleration_limits = np.array(
            [
                limits.a_xy,
                limits.a_xy,
                limits.a_z_up if z_positive_direction else limits.a_z_down,
            ],
            dtype=np.float64,
        )
        active_jerk_limits = np.array(
            [
                limits.j_xy,
                limits.j_xy,
                limits.j_z_up if z_positive_direction else limits.j_z_down,
            ],
            dtype=np.float64,
        )

        direction = scaled_action if deadman_active else -self.cmd_velocity
        workspace_scale = self._workspace_slowdown_scale(
            target_position, direction, workspace_low, workspace_high
        )
        desired_velocity = np.zeros(3, dtype=np.float64)
        if deadman_active:
            desired_velocity = self._action_to_user_velocity(
                scaled_action, limits, workspace_scale
            )
        desired_velocity = self._limit_axis_velocity(desired_velocity, limits)
        user_velocity = desired_velocity.copy()

        old_velocity = self.cmd_velocity.copy()
        raw_acceleration = (desired_velocity - old_velocity) / dt_used
        desired_acceleration = self._limit_axis_acceleration(raw_acceleration, limits)
        accel_limited = np.abs(raw_acceleration - desired_acceleration) > 1e-15

        acceleration_delta = desired_acceleration - self.cmd_acceleration
        limited_acceleration_delta = self._limit_axis_jerk_delta(
            acceleration_delta, limits, dt_used
        )
        jerk_limited = np.abs(acceleration_delta - limited_acceleration_delta) > 1e-15
        self.cmd_acceleration += limited_acceleration_delta
        pre_cmd_accel_limit = self.cmd_acceleration.copy()
        self.cmd_acceleration = self._limit_axis_acceleration(self.cmd_acceleration, limits)
        accel_limited |= np.abs(pre_cmd_accel_limit - self.cmd_acceleration) > 1e-15

        candidate_velocity = old_velocity + self.cmd_acceleration * dt_used
        candidate_velocity = self._avoid_velocity_target_overshoot(
            old_velocity, candidate_velocity, desired_velocity
        )
        pre_velocity_limit_velocity = candidate_velocity.copy()
        self.cmd_velocity = self._limit_axis_velocity(candidate_velocity, limits)
        post_velocity_limit_velocity = self.cmd_velocity.copy()
        velocity_limited = np.abs(pre_velocity_limit_velocity - post_velocity_limit_velocity) > 1e-15
        self.cmd_acceleration = (self.cmd_velocity - old_velocity) / dt_used
        desired_acceleration = self.cmd_acceleration.copy()

        pre_guard_velocity = self.cmd_velocity.copy()
        raw_delta_position = self.cmd_velocity * dt_used
        delta_position = self._limit_step_delta(raw_delta_position)
        delta_limited = np.abs(raw_delta_position - delta_position) > 1e-15
        next_position = target_position + delta_position
        pre_guard_target = next_position.copy()
        delta_position, workspace_clamped = self._limit_delta_to_workspace(
            target_position, next_position, workspace_low, workspace_high
        )
        delta_limited |= workspace_clamped
        self.cmd_velocity[workspace_clamped] = 0.0
        self.cmd_acceleration[workspace_clamped] = 0.0
        post_guard_target = target_position + delta_position

        return MotionStepResult(
            delta_position=delta_position,
            dt_used=dt_used,
            braking=not deadman_active,
            stale_dt=stale_dt,
            workspace_clamped=workspace_clamped,
            workspace_slowdown_scale=workspace_scale,
            z_at_upper_bound=bool(target_position[2] >= workspace_high[2]),
            z_at_lower_bound=bool(target_position[2] <= workspace_low[2]),
            pre_guard_velocity=pre_guard_velocity,
            post_guard_velocity=self.cmd_velocity.copy(),
            pre_guard_target=pre_guard_target,
            post_guard_target=post_guard_target,
            scaled_action=scaled_action,
            input_after_deadzone=input_after_deadzone,
            u_norm_before_clip=u_norm_before_clip,
            u_norm_after_clip=u_norm_after_clip,
            active_velocity_limits=active_velocity_limits,
            active_acceleration_limits=active_acceleration_limits,
            active_jerk_limits=active_jerk_limits,
            user_acceleration=raw_acceleration,
            desired_acceleration=desired_acceleration,
            steady_velocity_estimate=desired_velocity,
            velocity_saturation_scale=workspace_scale,
            pre_velocity_limit_velocity=pre_velocity_limit_velocity,
            post_velocity_limit_velocity=post_velocity_limit_velocity,
            accel_limited=accel_limited,
            jerk_limited=jerk_limited,
            velocity_limited=velocity_limited,
            delta_limited=delta_limited,
            user_velocity=user_velocity,
            desired_velocity=desired_velocity,
        )

    def _effective_dt(self, dt: float) -> tuple[float, bool]:
        dt_nominal = max(1e-6, float(self.config.dt_nominal))
        dt_max = max(dt_nominal, float(self.config.dt_max))
        if not np.isfinite(dt) or dt <= 0.0 or dt > dt_max:
            return dt_nominal, True
        return float(dt), False

    def _mode_limits(self, fine_mode: bool) -> AxisMotionLimits:
        limits = self.config.fine_limits if fine_mode else self.config.coarse_limits
        return limits.scaled(self.config.speed_scale)

    def _scaled_action(self, action: np.ndarray) -> np.ndarray:
        return self._scaled_action_with_debug(action)[1]

    def _scaled_action_with_debug(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, float, float]:
        input_after_deadzone = np.array(
            [
                deadzone_normalized(v, self.config.translation_deadzone)
                for v in action
            ],
            dtype=np.float64,
        )
        processed = np.array(
            [
                math.copysign(abs(v) ** max(1.0, float(self.config.input_power)), v)
                if v != 0.0
                else 0.0
                for v in input_after_deadzone
            ],
            dtype=np.float64,
        )
        max_action_norm = max(1e-6, float(self.config.max_action_norm))
        u_norm_before_clip = float(np.linalg.norm(processed))
        if u_norm_before_clip > max_action_norm and u_norm_before_clip > 1e-12:
            processed *= max_action_norm / u_norm_before_clip
        u_norm_after_clip = float(np.linalg.norm(processed))
        return input_after_deadzone, processed, u_norm_before_clip, u_norm_after_clip

    def _action_to_user_velocity(
        self, scaled_action: np.ndarray, limits: AxisMotionLimits, workspace_scale: np.ndarray
    ) -> np.ndarray:
        velocity = np.zeros(3, dtype=np.float64)
        velocity[0] = scaled_action[0] * limits.v_xy
        velocity[1] = scaled_action[1] * limits.v_xy
        z_limit = limits.v_z_up if scaled_action[2] >= 0.0 else limits.v_z_down
        velocity[2] = scaled_action[2] * z_limit
        return velocity * workspace_scale

    def _workspace_slowdown_scale(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        workspace_low: np.ndarray,
        workspace_high: np.ndarray,
    ) -> np.ndarray:
        slow_distance = max(1e-9, float(self.config.workspace_slowdown_distance))
        scale = np.ones(3, dtype=np.float64)
        for i in range(3):
            if direction[i] > 0.0 or self.cmd_velocity[i] > 0.0:
                distance = workspace_high[i] - position[i]
            elif direction[i] < 0.0 or self.cmd_velocity[i] < 0.0:
                distance = position[i] - workspace_low[i]
            else:
                continue
            scale[i] = float(np.clip(distance / slow_distance, 0.0, 1.0))
        return scale

    def _limit_axis_velocity(
        self, velocity: np.ndarray, limits: AxisMotionLimits
    ) -> np.ndarray:
        limited = velocity.copy()
        limited[0] = float(np.clip(limited[0], -limits.v_xy, limits.v_xy))
        limited[1] = float(np.clip(limited[1], -limits.v_xy, limits.v_xy))
        limited[2] = float(np.clip(limited[2], -limits.v_z_down, limits.v_z_up))
        return limited

    def _limit_axis_acceleration(
        self, acceleration: np.ndarray, limits: AxisMotionLimits
    ) -> np.ndarray:
        limited = acceleration.copy()
        limited[0] = float(np.clip(limited[0], -limits.a_xy, limits.a_xy))
        limited[1] = float(np.clip(limited[1], -limits.a_xy, limits.a_xy))
        limited[2] = float(np.clip(limited[2], -limits.a_z_down, limits.a_z_up))
        return limited

    def _limit_axis_jerk_delta(
        self, acceleration_delta: np.ndarray, limits: AxisMotionLimits, dt: float
    ) -> np.ndarray:
        limited = acceleration_delta.copy()
        xy_delta = limits.j_xy * dt
        z_up_delta = limits.j_z_up * dt
        z_down_delta = limits.j_z_down * dt
        limited[0] = float(np.clip(limited[0], -xy_delta, xy_delta))
        limited[1] = float(np.clip(limited[1], -xy_delta, xy_delta))
        limited[2] = float(np.clip(limited[2], -z_down_delta, z_up_delta))
        return limited

    @staticmethod
    def _avoid_velocity_target_overshoot(
        old_velocity: np.ndarray, candidate_velocity: np.ndarray, desired_velocity: np.ndarray
    ) -> np.ndarray:
        limited = candidate_velocity.copy()
        for i in range(3):
            if old_velocity[i] <= desired_velocity[i] <= candidate_velocity[i]:
                limited[i] = desired_velocity[i]
            elif old_velocity[i] >= desired_velocity[i] >= candidate_velocity[i]:
                limited[i] = desired_velocity[i]
        return limited

    def _limit_step_delta(self, delta: np.ndarray) -> np.ndarray:
        limited = delta.copy()
        limited[0] = float(np.clip(limited[0], -self.config.delta_limits.xy, self.config.delta_limits.xy))
        limited[1] = float(np.clip(limited[1], -self.config.delta_limits.xy, self.config.delta_limits.xy))
        limited[2] = float(
            np.clip(
                limited[2],
                -self.config.delta_limits.z_down,
                self.config.delta_limits.z_up,
            )
        )
        return limited

    def _limit_delta_to_workspace(
        self,
        position: np.ndarray,
        next_position: np.ndarray,
        workspace_low: np.ndarray,
        workspace_high: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        raw_delta = next_position - position
        limited_delta = raw_delta.copy()
        clamped = np.zeros(3, dtype=bool)
        for i in range(3):
            if position[i] < workspace_low[i]:
                if raw_delta[i] <= 0.0:
                    limited_delta[i] = 0.0
            elif position[i] > workspace_high[i]:
                if raw_delta[i] >= 0.0:
                    limited_delta[i] = 0.0
            elif next_position[i] < workspace_low[i]:
                limited_delta[i] = workspace_low[i] - position[i]
            elif next_position[i] > workspace_high[i]:
                limited_delta[i] = workspace_high[i] - position[i]

            if limited_delta[i] != raw_delta[i]:
                clamped[i] = True
        return limited_delta, clamped
