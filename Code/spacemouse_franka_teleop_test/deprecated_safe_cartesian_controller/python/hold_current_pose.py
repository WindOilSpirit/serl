#!/usr/bin/env python3
"""Continuously publish the current Franka pose back to the controller."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


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
    norm = np.linalg.norm(quat)
    if norm < 1e-9:
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    else:
        quat = quat / norm
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


class HoldCurrentPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("spacemouse_franka_hold_current_pose_test")
        self.declare_parameter("target_topic", "/serl_safe_cartesian_pose_controller/target_pose")
        self.declare_parameter("current_pose_topic", "/franka_robot_state_broadcaster/current_pose")
        self.declare_parameter("frame_id", "base")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("log_rate_hz", 2.0)

        self.target_topic = self.get_parameter("target_topic").value
        self.current_pose_topic = self.get_parameter("current_pose_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.log_rate_hz = float(self.get_parameter("log_rate_hz").value)

        self.current_pose = None
        self.last_log_time = 0.0

        self.target_pub = self.create_publisher(PoseStamped, self.target_topic, 10)
        self.create_subscription(PoseStamped, self.current_pose_topic, self._pose_cb, 10)
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)

        self.get_logger().info("Hold-current-pose test started")

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.current_pose = pose_msg_to_array(msg)

    def _tick(self) -> None:
        now = time.time()
        if self.current_pose is None:
            if now - self.last_log_time > 1.0 / max(0.1, self.log_rate_hz):
                self.last_log_time = now
                self.get_logger().info("waiting for current pose")
            return

        self.target_pub.publish(array_to_pose_msg(self.current_pose, self.frame_id, self))
        if now - self.last_log_time > 1.0 / max(0.1, self.log_rate_hz):
            self.last_log_time = now
            xyz = self.current_pose[:3]
            self.get_logger().info("holding pose xyz=%s" % np.array2string(xyz, precision=4, suppress_small=True))


def main() -> None:
    rclpy.init()
    node = HoldCurrentPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
