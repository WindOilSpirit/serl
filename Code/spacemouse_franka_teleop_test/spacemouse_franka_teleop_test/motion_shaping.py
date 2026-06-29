#!/usr/bin/env python3
"""Pure flight-style motion shaping for SpaceMouse Cartesian teleoperation."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


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


class MotionShaper:
    """Convert normalized SpaceMouse acceleration intent into target-position deltas."""

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
        workspace_scale = np.ones(3, dtype=np.float64)
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

        user_acceleration = np.zeros(3, dtype=np.float64)
        if deadman_active:
            user_acceleration = self._action_to_user_acceleration(
                scaled_action, limits, workspace_scale
            )
        damping = self.config.d_move if deadman_active else self.config.d_stop
        desired_acceleration = user_acceleration - max(0.0, damping) * self.cmd_velocity
        pre_accel_limit_desired = desired_acceleration.copy()
        desired_acceleration = self._limit_axis_acceleration(desired_acceleration, limits)
        accel_limited = np.abs(pre_accel_limit_desired - desired_acceleration) > 1e-15
        velocity_saturation_scale = self._velocity_saturation_scale(
            desired_acceleration, limits
        )
        desired_acceleration *= velocity_saturation_scale

        acceleration_delta = desired_acceleration - self.cmd_acceleration
        limited_acceleration_delta = self._limit_axis_jerk_delta(
            acceleration_delta, limits, dt_used
        )
        jerk_limited = np.abs(acceleration_delta - limited_acceleration_delta) > 1e-15
        self.cmd_acceleration += limited_acceleration_delta
        pre_cmd_accel_limit = self.cmd_acceleration.copy()
        self.cmd_acceleration = self._limit_axis_acceleration(self.cmd_acceleration, limits)
        accel_limited |= np.abs(pre_cmd_accel_limit - self.cmd_acceleration) > 1e-15

        self.cmd_velocity += self.cmd_acceleration * dt_used
        pre_velocity_limit_velocity = self.cmd_velocity.copy()
        self.cmd_velocity = self._limit_axis_velocity(self.cmd_velocity, limits)
        post_velocity_limit_velocity = self.cmd_velocity.copy()
        velocity_limited = np.abs(pre_velocity_limit_velocity - post_velocity_limit_velocity) > 1e-15

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
        if max(0.0, self.config.d_move) > 1e-12:
            steady_velocity_estimate = user_acceleration / self.config.d_move
        else:
            steady_velocity_estimate = np.full(3, np.nan, dtype=np.float64)
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
            user_acceleration=user_acceleration,
            desired_acceleration=desired_acceleration,
            steady_velocity_estimate=steady_velocity_estimate,
            velocity_saturation_scale=velocity_saturation_scale,
            pre_velocity_limit_velocity=pre_velocity_limit_velocity,
            post_velocity_limit_velocity=post_velocity_limit_velocity,
            accel_limited=accel_limited,
            jerk_limited=jerk_limited,
            velocity_limited=velocity_limited,
            delta_limited=delta_limited,
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

    def _action_to_user_acceleration(
        self, scaled_action: np.ndarray, limits: AxisMotionLimits, workspace_scale: np.ndarray
    ) -> np.ndarray:
        acceleration = np.zeros(3, dtype=np.float64)
        acceleration[0] = scaled_action[0] * limits.a_xy
        acceleration[1] = scaled_action[1] * limits.a_xy
        z_limit = limits.a_z_up if scaled_action[2] >= 0.0 else limits.a_z_down
        acceleration[2] = scaled_action[2] * z_limit
        return acceleration * workspace_scale

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

    def _velocity_saturation_scale(
        self, desired_acceleration: np.ndarray, limits: AxisMotionLimits
    ) -> np.ndarray:
        lower_velocity = np.array(
            [-limits.v_xy, -limits.v_xy, -limits.v_z_down],
            dtype=np.float64,
        )
        upper_velocity = np.array(
            [limits.v_xy, limits.v_xy, limits.v_z_up],
            dtype=np.float64,
        )
        scale = np.ones(3, dtype=np.float64)
        for i in range(3):
            if desired_acceleration[i] > 0.0:
                headroom = upper_velocity[i] - self.cmd_velocity[i]
                band = self._velocity_soft_band(i, True, limits)
                scale[i] = self._smoothstep01(headroom / band)
            elif desired_acceleration[i] < 0.0:
                headroom = self.cmd_velocity[i] - lower_velocity[i]
                band = self._velocity_soft_band(i, False, limits)
                scale[i] = self._smoothstep01(headroom / band)
        return scale

    def _velocity_soft_band(
        self, axis: int, positive_direction: bool, limits: AxisMotionLimits
    ) -> float:
        if axis < 2:
            acceleration_limit = limits.a_xy
            reducing_jerk_limit = limits.j_xy
        elif positive_direction:
            acceleration_limit = limits.a_z_up
            reducing_jerk_limit = limits.j_z_down
        else:
            acceleration_limit = limits.a_z_down
            reducing_jerk_limit = limits.j_z_up

        if positive_direction:
            acceleration_limit = max(acceleration_limit, self.cmd_acceleration[axis])
        else:
            acceleration_limit = max(acceleration_limit, -self.cmd_acceleration[axis])
        reducing_jerk_limit = max(1e-9, reducing_jerk_limit)
        return max(1e-9, acceleration_limit * acceleration_limit / (2.0 * reducing_jerk_limit))

    @staticmethod
    def _smoothstep01(value: float) -> float:
        x = float(np.clip(value, 0.0, 1.0))
        return x * x * (3.0 - 2.0 * x)

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
