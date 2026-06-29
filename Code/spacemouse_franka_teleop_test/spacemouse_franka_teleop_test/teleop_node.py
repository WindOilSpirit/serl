#!/usr/bin/env python3
"""Standalone SpaceMouse -> Franka teleoperation test node."""

from __future__ import annotations

import threading
import time
import math
from dataclasses import dataclass

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool


@dataclass
class SpaceMouseSample:
    action: np.ndarray
    buttons: list[int]
    timestamp: float


class SpaceMouseReader:
    def __init__(self, allow_missing: bool = False) -> None:
        try:
            import pyspacemouse  # type: ignore
        except Exception as exc:
            if not allow_missing:
                raise RuntimeError("pyspacemouse import failed") from exc
            self._device = None
            self._lock = threading.Lock()
            self._sample = SpaceMouseSample(np.zeros(6, dtype=np.float64), [0, 0], time.time())
            self._stop = threading.Event()
            return
        self._lock = threading.Lock()
        self._sample = SpaceMouseSample(np.zeros(6, dtype=np.float64), [0, 0], time.time())
        self._stop = threading.Event()
        self._device = None
        try:
            self._device = pyspacemouse.open()
        except Exception:
            self._device = None
        if self._device is None:
            if not allow_missing:
                raise RuntimeError("pyspacemouse.open() failed")
            return
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if hasattr(self, "_thread"):
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
        if self._device is None:
            return
        while not self._stop.is_set():
            state = self._device.read()
            if state is None:
                time.sleep(0.002)
                continue
            action = np.array(
                [-state.y, state.x, state.z, -state.roll, -state.pitch, -state.yaw],
                dtype=np.float64,
            )
            with self._lock:
                self._sample = SpaceMouseSample(action, list(state.buttons), time.time())


def get_double_array(node: Node, name: str) -> np.ndarray:
    return np.asarray(node.get_parameter(name).value, dtype=np.float64)


class SpaceMouseFrankaTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("spacemouse_franka_teleop_test")

        self.declare_parameter("target_topic", "/spacemouse_franka_teleop/action")
        self.declare_parameter("deadman_topic", "/spacemouse_franka_teleop/deadman")
        self.declare_parameter("retreat_topic", "/spacemouse_franka_teleop/retreat")
        self.declare_parameter("fine_mode_topic", "/spacemouse_franka_teleop/fine_mode")
        self.declare_parameter("controller_manager_service", "/controller_manager/list_controllers")
        self.declare_parameter("controller_name", "serl_cartesian_impedance_controller")
        self.declare_parameter("require_active_controller", False)
        self.declare_parameter("debug_simulation_mode", False)
        self.declare_parameter("frame_id", "base")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("log_rate_hz", 2.0)
        self.declare_parameter("deadman_button_index", 0)
        self.declare_parameter("fine_button_index", 1)
        self.declare_parameter("retreat_button_index", -1)
        self.declare_parameter("command_start_hold_s", 0.4)
        self.declare_parameter("debug_force_deadman_after_sec", 0.0)
        self.declare_parameter("debug_z_input_after_deadman_sec", 4.0)
        self.declare_parameter("debug_z_input_value", 0.15)
        self.declare_parameter("debug_z_input_duration_sec", 2.0)
        self.declare_parameter("debug_z_input_ramp_s", 0.4)
        self.declare_parameter("debug_z_oscillation_mode", False)
        self.declare_parameter("debug_z_cycle_period_s", 6.0)
        self.declare_parameter("debug_z_cycle_delay_s", 0.5)
        self.declare_parameter("debug_z_cycle_value", 0.15)

        self.declare_parameter("axis_sign", [1.0, 1.0, 1.0])

        self.target_topic = self.get_parameter("target_topic").value
        self.deadman_topic = self.get_parameter("deadman_topic").value
        self.retreat_topic = self.get_parameter("retreat_topic").value
        self.fine_mode_topic = self.get_parameter("fine_mode_topic").value
        self.controller_manager_service = self.get_parameter("controller_manager_service").value
        self.controller_name = self.get_parameter("controller_name").value
        self.require_active_controller = bool(self.get_parameter("require_active_controller").value)
        self.debug_simulation_mode = bool(self.get_parameter("debug_simulation_mode").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.log_rate_hz = float(self.get_parameter("log_rate_hz").value)
        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.fine_button_index = int(self.get_parameter("fine_button_index").value)
        self.retreat_button_index = int(self.get_parameter("retreat_button_index").value)
        self.command_start_hold_s = max(0.0, float(self.get_parameter("command_start_hold_s").value))
        self.debug_force_deadman_after_sec = float(
            self.get_parameter("debug_force_deadman_after_sec").value
        )
        self.debug_z_input_after_deadman_sec = float(
            self.get_parameter("debug_z_input_after_deadman_sec").value
        )
        self.debug_z_input_value = float(self.get_parameter("debug_z_input_value").value)
        self.debug_z_input_duration_sec = float(
            self.get_parameter("debug_z_input_duration_sec").value
        )
        self.debug_z_input_ramp_s = max(0.0, float(self.get_parameter("debug_z_input_ramp_s").value))
        self.debug_z_oscillation_mode = bool(self.get_parameter("debug_z_oscillation_mode").value)
        self.debug_z_cycle_period_s = max(1e-3, float(self.get_parameter("debug_z_cycle_period_s").value))
        self.debug_z_cycle_delay_s = max(0.0, float(self.get_parameter("debug_z_cycle_delay_s").value))
        self.debug_z_cycle_value = float(self.get_parameter("debug_z_cycle_value").value)
        self.axis_sign = get_double_array(self, "axis_sign")

        self.last_action = np.zeros(3, dtype=np.float64)
        self.control_start_time: float | None = None
        self.previous_deadman = False
        self.last_log_time = 0.0
        self.start_time = time.time()
        self.last_timer_time = time.time()
        self.controller_active = self.debug_simulation_mode or not self.require_active_controller
        self.controller_state = "unchecked"
        self.last_controller_check_time = 0.0
        self.controller_check_period_s = 1.0
        self.pending_controller_future = None

        self.target_pub = self.create_publisher(TwistStamped, self.target_topic, 10)
        self.deadman_pub = self.create_publisher(Bool, self.deadman_topic, 10)
        self.retreat_pub = self.create_publisher(Bool, self.retreat_topic, 10)
        self.fine_mode_pub = self.create_publisher(Bool, self.fine_mode_topic, 10)
        self.controller_client = self.create_client(ListControllers, self.controller_manager_service)
        self.spacemouse = SpaceMouseReader(allow_missing=self.debug_simulation_mode)
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)

        self.get_logger().info("SpaceMouse Franka action source started")
        self.get_logger().info(f"  target_topic: {self.target_topic}")
        self.get_logger().info(f"  deadman_topic: {self.deadman_topic}")
        self.get_logger().info(f"  retreat_topic: {self.retreat_topic}")
        self.get_logger().info(
            "  downstream robot server/controller integration is intentionally not configured here"
        )
        self.get_logger().info(f"  debug_simulation_mode: {self.debug_simulation_mode}")
        self.get_logger().info(f"  debug_z_oscillation_mode: {self.debug_z_oscillation_mode}")

    def destroy_node(self) -> bool:
        if hasattr(self, "spacemouse"):
            self.spacemouse.close()
        return super().destroy_node()

    def _button(self, buttons: list[int], index: int) -> bool:
        return 0 <= index < len(buttons) and bool(buttons[index])

    def _tick(self) -> None:
        now = time.time()
        dt = max(1e-3, min(0.2, now - self.last_timer_time))
        self.last_timer_time = now
        self._update_controller_status(now)

        sample = self.spacemouse.latest()
        deadman = self._button(sample.buttons, self.deadman_button_index)
        fine = self._button(sample.buttons, self.fine_button_index)
        retreat = self._button(sample.buttons, self.retreat_button_index)
        if self.debug_force_deadman_after_sec > 0.0 and now - self.start_time >= self.debug_force_deadman_after_sec:
            deadman = True

        if deadman != self.previous_deadman:
            self.last_action[:] = 0.0
        if deadman and not self.previous_deadman:
            self.control_start_time = now
        elif not deadman:
            self.control_start_time = None
        self.previous_deadman = deadman
        self._publish_input_flags(deadman, retreat, fine)

        action = self._effective_translation_input(sample, deadman, now)
        self.last_action[:] = action

        self._publish_target(action, sample.action[3:6])
        note = "input facts"
        if not self.debug_simulation_mode and self.require_active_controller and not self.controller_active:
            note = f"{note}; controller not active ({self.controller_state})"
        if not self.debug_simulation_mode and self.target_pub.get_subscription_count() == 0:
            note = f"{note}; no action server subscription"
        self._log_throttled(now, sample, deadman, fine, action, action, note)

    def _effective_translation_input(self, sample: SpaceMouseSample, deadman: bool, now: float) -> np.ndarray:
        raw_translation = sample.action[:3].copy()
        if (
            deadman
            and self.control_start_time is not None
            and self.debug_z_input_after_deadman_sec > 0.0
            and self.debug_z_input_duration_sec > 0.0
        ):
            elapsed = now - self.control_start_time
            window_start = self.debug_z_input_after_deadman_sec
            window_end = window_start + self.debug_z_input_duration_sec
            if window_start <= elapsed < window_end:
                raw_translation[:] = 0.0
                if self.debug_z_oscillation_mode:
                    raw_translation[2] = self._debug_z_oscillation(elapsed - window_start)
                else:
                    ramp = min(self.debug_z_input_ramp_s, self.debug_z_input_duration_sec / 2.0)
                    if ramp > 0.0:
                        in_ramp = min(1.0, max(0.0, (elapsed - window_start) / ramp))
                        out_ramp = min(1.0, max(0.0, (window_end - elapsed) / ramp))
                        envelope = min(in_ramp, out_ramp)
                    else:
                        envelope = 1.0
                    raw_translation[2] = float(np.clip(self.debug_z_input_value, -1.0, 1.0)) * envelope
        return raw_translation * self.axis_sign

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

    def _debug_z_oscillation(self, t: float) -> float:
        if t < self.debug_z_cycle_delay_s:
            return 0.0
        t -= self.debug_z_cycle_delay_s
        phase = 2.0 * math.pi * (t / self.debug_z_cycle_period_s)
        return float(np.clip(self.debug_z_cycle_value * math.sin(phase), -1.0, 1.0))

    def _publish_target(self, linear_velocity: np.ndarray, angular_velocity: np.ndarray) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(linear_velocity[0])
        msg.twist.linear.y = float(linear_velocity[1])
        msg.twist.linear.z = float(linear_velocity[2])
        msg.twist.angular.x = float(angular_velocity[0])
        msg.twist.angular.y = float(angular_velocity[1])
        msg.twist.angular.z = float(angular_velocity[2])
        self.target_pub.publish(msg)

    def _publish_input_flags(self, deadman: bool, retreat: bool, fine: bool) -> None:
        deadman_msg = Bool()
        deadman_msg.data = bool(deadman)
        self.deadman_pub.publish(deadman_msg)

        retreat_msg = Bool()
        retreat_msg.data = bool(retreat)
        self.retreat_pub.publish(retreat_msg)

        fine_msg = Bool()
        fine_msg.data = bool(fine)
        self.fine_mode_pub.publish(fine_msg)

    def _log_throttled(
        self,
        now: float,
        sample: SpaceMouseSample,
        deadman: bool,
        fine: bool,
        command_delta: np.ndarray,
        effective_translation: np.ndarray,
        reason: str,
    ) -> None:
        if now - self.last_log_time < 1.0 / max(0.1, self.log_rate_hz):
            return
        self.last_log_time = now
        mode = "fine" if fine else "coarse"
        self.get_logger().info(
            "state=%s deadman=%s mode=%s raw=%s action=%s last_action=%s note=%s"
            % (
                "DEADMAN" if deadman else "IDLE",
                deadman,
                mode,
                np.array2string(sample.action[:3], precision=3, suppress_small=True),
                np.array2string(command_delta, precision=5, suppress_small=True),
                np.array2string(self.last_action, precision=5, suppress_small=True),
                reason,
            )
        )
        self.get_logger().info(
            "effective_translation=%s"
            % np.array2string(effective_translation, precision=4, suppress_small=True)
        )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = SpaceMouseFrankaTeleopNode()
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
