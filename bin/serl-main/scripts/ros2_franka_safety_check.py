#!/usr/bin/env python3
"""Read-only ROS2 Franka safety check.

This script does not send any robot command. It verifies that the ROS2 Franka
bringup is publishing state and that controller_manager is reachable before any
SERL control bridge is allowed to run.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class LatestState:
    pose: Optional[PoseStamped] = None
    wrench: Optional[WrenchStamped] = None
    joints: Optional[JointState] = None


def ns_join(namespace: str, suffix: str) -> str:
    namespace = namespace.strip("/")
    suffix = suffix.strip("/")
    return f"/{namespace}/{suffix}" if namespace else f"/{suffix}"


class FrankaSafetyCheck(Node):
    def __init__(self, namespace: str):
        super().__init__("serl_franka_safety_check")
        self.state = LatestState()
        self.namespace = namespace
        self.current_pose_topic = ns_join(
            namespace, "franka_robot_state_broadcaster/current_pose"
        )
        self.wrench_topic = ns_join(
            namespace,
            "franka_robot_state_broadcaster/external_wrench_in_stiffness_frame",
        )
        self.joint_states_topic = ns_join(namespace, "joint_states")
        self.controller_service = ns_join(namespace, "controller_manager/list_controllers")

        self.create_subscription(PoseStamped, self.current_pose_topic, self._pose_cb, 10)
        self.create_subscription(WrenchStamped, self.wrench_topic, self._wrench_cb, 10)
        self.create_subscription(JointState, self.joint_states_topic, self._joints_cb, 10)
        self.controller_client = self.create_client(
            ListControllers, self.controller_service
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.state.pose = msg

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self.state.wrench = msg

    def _joints_cb(self, msg: JointState) -> None:
        self.state.joints = msg

    def wait_for_state(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.pose and self.state.wrench and self.state.joints:
                return True
        return False

    def list_controllers(self, timeout: float):
        if not self.controller_client.wait_for_service(timeout_sec=timeout):
            return None
        future = self.controller_client.call_async(ListControllers.Request())
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            return None
        return future.result().controller


def wrench_norm(wrench: WrenchStamped) -> tuple[float, float]:
    force = wrench.wrench.force
    torque = wrench.wrench.torque
    f_norm = math.sqrt(force.x**2 + force.y**2 + force.z**2)
    t_norm = math.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
    return f_norm, t_norm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read-only ROS2 Franka safety check")
    parser.add_argument("--namespace", default="", help="Franka ROS2 namespace")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-force-n", type=float, default=25.0)
    parser.add_argument("--max-torque-nm", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = FrankaSafetyCheck(args.namespace)
    try:
        print("Checking topics:")
        print(f"  pose:   {node.current_pose_topic}")
        print(f"  wrench: {node.wrench_topic}")
        print(f"  joints: {node.joint_states_topic}")
        print(f"Checking service: {node.controller_service}")

        if not node.wait_for_state(args.timeout):
            print("ERROR: timed out waiting for pose, wrench, and joint states.")
            return 2

        pose = node.state.pose.pose
        f_norm, t_norm = wrench_norm(node.state.wrench)
        print(
            "Current pose xyz/quaternion: "
            f"[{pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}], "
            f"[{pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f}]"
        )
        print(f"External wrench norms: force={f_norm:.3f} N, torque={t_norm:.3f} Nm")
        print(f"Joint state names: {', '.join(node.state.joints.name)}")

        controllers = node.list_controllers(args.timeout)
        if controllers is None:
            print("ERROR: controller_manager list_controllers service not reachable.")
            return 3
        print("Controllers:")
        for controller in controllers:
            print(f"  {controller.name}: {controller.state}")

        if f_norm > args.max_force_n:
            print(f"ERROR: force norm exceeds threshold {args.max_force_n} N.")
            return 4
        if t_norm > args.max_torque_nm:
            print(f"ERROR: torque norm exceeds threshold {args.max_torque_nm} Nm.")
            return 5

        print("Read-only Franka safety check passed.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
