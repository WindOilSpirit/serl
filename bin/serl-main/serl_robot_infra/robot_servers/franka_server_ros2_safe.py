#!/usr/bin/env python3
"""Safety-gated ROS2 Franka HTTP server for SERL.

This server preserves the HTTP surface used by SERL's FrankaEnv while using the
ROS2 Franka stack. Motion is disabled by default. Enable it explicitly with
--enable_motion after running the read-only safety check.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from flask import Flask, jsonify, request
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState


def ns_join(namespace: str, suffix: str) -> str:
    namespace = namespace.strip("/")
    suffix = suffix.strip("/")
    return f"/{namespace}/{suffix}" if namespace else f"/{suffix}"


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


def wrench_norm(msg: WrenchStamped) -> tuple[float, float]:
    force = msg.wrench.force
    torque = msg.wrench.torque
    f_norm = math.sqrt(force.x**2 + force.y**2 + force.z**2)
    t_norm = math.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
    return f_norm, t_norm


def make_pose_msg(pose: np.ndarray, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(pose[0])
    msg.pose.position.y = float(pose[1])
    msg.pose.position.z = float(pose[2])
    quat = np.asarray(pose[3:7], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < 1e-9:
        raise ValueError("Quaternion norm is too small")
    quat = quat / norm
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


@dataclass
class SafeLimits:
    max_command_delta_m: float = 0.002
    max_command_delta_rad: float = 0.02
    max_force_n: float = 25.0
    max_torque_nm: float = 8.0
    workspace_low: np.ndarray = field(
        default_factory=lambda: np.array([0.25, -0.45, 0.02], dtype=np.float64)
    )
    workspace_high: np.ndarray = field(
        default_factory=lambda: np.array([0.75, 0.45, 0.65], dtype=np.float64)
    )


class SafeFrankaRos2Node(Node):
    def __init__(
        self,
        namespace: str,
        controller_name: str,
        target_topic: str,
        frame_id: str,
        limits: SafeLimits,
        motion_enabled: bool,
    ):
        super().__init__("serl_safe_franka_http_bridge")
        self.namespace = namespace
        self.controller_name = controller_name
        self.frame_id = frame_id
        self.limits = limits
        self.motion_enabled = motion_enabled
        self._lock = threading.Lock()
        self._last_pose: Optional[PoseStamped] = None
        self._last_desired_pose: Optional[PoseStamped] = None
        self._last_wrench: Optional[WrenchStamped] = None
        self._last_measured_joints: Optional[JointState] = None
        self._last_joint_states: Optional[JointState] = None
        self._last_command_time = 0.0
        self._fault = ""

        self.current_pose_topic = ns_join(
            namespace, "franka_robot_state_broadcaster/current_pose"
        )
        self.last_desired_pose_topic = ns_join(
            namespace, "franka_robot_state_broadcaster/last_desired_pose"
        )
        self.wrench_topic = ns_join(
            namespace,
            "franka_robot_state_broadcaster/external_wrench_in_stiffness_frame",
        )
        self.measured_joints_topic = ns_join(
            namespace, "franka_robot_state_broadcaster/measured_joint_states"
        )
        self.joint_states_topic = ns_join(namespace, "joint_states")
        self.controller_service = ns_join(namespace, "controller_manager/list_controllers")
        self.target_topic = ns_join(namespace, target_topic)

        self.create_subscription(PoseStamped, self.current_pose_topic, self._pose_cb, 10)
        self.create_subscription(
            PoseStamped, self.last_desired_pose_topic, self._desired_pose_cb, 10
        )
        self.create_subscription(WrenchStamped, self.wrench_topic, self._wrench_cb, 10)
        self.create_subscription(
            JointState, self.measured_joints_topic, self._measured_joints_cb, 10
        )
        self.create_subscription(JointState, self.joint_states_topic, self._joints_cb, 10)
        self.target_pub = self.create_publisher(PoseStamped, self.target_topic, 10)
        self.controller_client = self.create_client(
            ListControllers, self.controller_service
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            self._last_pose = msg

    def _desired_pose_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            self._last_desired_pose = msg

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        with self._lock:
            self._last_wrench = msg

    def _measured_joints_cb(self, msg: JointState) -> None:
        with self._lock:
            self._last_measured_joints = msg

    def _joints_cb(self, msg: JointState) -> None:
        with self._lock:
            self._last_joint_states = msg

    def wait_for_state(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                ready = self._last_pose is not None and self._last_wrench is not None
            if ready:
                return True
            time.sleep(0.05)
        return False

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            pose = self._last_pose
            desired_pose = self._last_desired_pose
            wrench = self._last_wrench
            measured = self._last_measured_joints
            joints = self._last_joint_states
            fault = self._fault

        current_pose = pose_msg_to_array(pose) if pose else np.zeros((7,))
        if desired_pose:
            desired = pose_msg_to_array(desired_pose)
            vel = current_pose[:6] - desired[:6]
        else:
            vel = np.zeros((6,))
        if wrench:
            force_vec = wrench.wrench.force
            torque_vec = wrench.wrench.torque
            force = np.array([force_vec.x, force_vec.y, force_vec.z])
            torque = np.array([torque_vec.x, torque_vec.y, torque_vec.z])
            f_norm, t_norm = wrench_norm(wrench)
        else:
            force = np.zeros((3,))
            torque = np.zeros((3,))
            f_norm, t_norm = 0.0, 0.0

        joint_msg = measured or joints
        q = np.asarray(joint_msg.position[:7], dtype=np.float64) if joint_msg else np.zeros((7,))
        dq = (
            np.asarray(joint_msg.velocity[:7], dtype=np.float64)
            if joint_msg and joint_msg.velocity
            else np.zeros((7,))
        )

        return {
            "pose": current_pose,
            "vel": vel,
            "force": force,
            "torque": torque,
            "force_norm": f_norm,
            "torque_norm": t_norm,
            "q": q,
            "dq": dq,
            "jacobian": np.zeros((6, 7), dtype=np.float64),
            "fault": fault,
            "motion_enabled": self.motion_enabled,
            "last_command_time": self._last_command_time,
        }

    def _set_fault(self, reason: str) -> None:
        with self._lock:
            self._fault = reason

    def clear_fault(self) -> None:
        with self._lock:
            self._fault = ""

    def controller_active(self, timeout: float = 0.5) -> bool:
        if not self.controller_client.wait_for_service(timeout_sec=timeout):
            self._set_fault("controller_manager service unavailable")
            return False
        future = self.controller_client.call_async(ListControllers.Request())
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            self._set_fault("controller_manager list timeout")
            return False
        for controller in future.result().controller:
            if controller.name == self.controller_name:
                if controller.state == "active":
                    return True
                self._set_fault(f"{self.controller_name} is {controller.state}")
                return False
        self._set_fault(f"{self.controller_name} not loaded")
        return False

    def validate_and_publish_pose(self, requested_pose: np.ndarray) -> tuple[bool, str]:
        if not self.motion_enabled:
            return False, "motion disabled; restart with --enable_motion to command robot"
        if requested_pose.shape != (7,) or not np.all(np.isfinite(requested_pose)):
            return False, "pose must be finite xyz+quaternion length 7"
        if not self.controller_active():
            return False, self.snapshot()["fault"]

        snap = self.snapshot()
        current = np.asarray(snap["pose"], dtype=np.float64)
        if not np.all(np.isfinite(current)) or np.linalg.norm(current[3:]) < 1e-9:
            return False, "current pose unavailable"
        if snap["force_norm"] > self.limits.max_force_n:
            return False, f"force {snap['force_norm']:.3f} exceeds threshold"
        if snap["torque_norm"] > self.limits.max_torque_nm:
            return False, f"torque {snap['torque_norm']:.3f} exceeds threshold"

        clipped_xyz = np.clip(
            requested_pose[:3], self.limits.workspace_low, self.limits.workspace_high
        )
        xyz_delta = clipped_xyz - current[:3]
        delta_norm = np.linalg.norm(xyz_delta)
        if delta_norm > self.limits.max_command_delta_m:
            xyz_delta *= self.limits.max_command_delta_m / delta_norm
        command_pose = current.copy()
        command_pose[:3] = current[:3] + xyz_delta

        current_rot = Rotation.from_quat(current[3:])
        target_rot = Rotation.from_quat(requested_pose[3:] / np.linalg.norm(requested_pose[3:]))
        delta_rot = target_rot * current_rot.inv()
        rotvec = delta_rot.as_rotvec()
        angle = np.linalg.norm(rotvec)
        if angle > self.limits.max_command_delta_rad and angle > 1e-12:
            rotvec *= self.limits.max_command_delta_rad / angle
        command_pose[3:] = (Rotation.from_rotvec(rotvec) * current_rot).as_quat()

        msg = make_pose_msg(command_pose, self.frame_id)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.target_pub.publish(msg)
        self._last_command_time = time.time()
        return True, "published clipped safe pose"


def spin_ros(node: SafeFrankaRos2Node) -> None:
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SERL-compatible safe ROS2 Franka HTTP server")
    parser.add_argument("--namespace", default="", help="Franka ROS2 namespace")
    parser.add_argument("--controller-name", default="serl_safe_cartesian_pose_controller")
    parser.add_argument(
        "--target-topic",
        default="serl_safe_cartesian_pose_controller/target_pose",
        help="Target topic relative to namespace",
    )
    parser.add_argument("--frame-id", default="fr3_link0")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--enable_motion", action="store_true")
    parser.add_argument("--max-command-delta-m", type=float, default=0.002)
    parser.add_argument("--max-command-delta-rad", type=float, default=0.02)
    parser.add_argument("--max-force-n", type=float, default=25.0)
    parser.add_argument("--max-torque-nm", type=float, default=8.0)
    parser.add_argument("--workspace-low", nargs=3, type=float, default=[0.25, -0.45, 0.02])
    parser.add_argument("--workspace-high", nargs=3, type=float, default=[0.75, 0.45, 0.65])
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    limits = SafeLimits(
        max_command_delta_m=args.max_command_delta_m,
        max_command_delta_rad=args.max_command_delta_rad,
        max_force_n=args.max_force_n,
        max_torque_nm=args.max_torque_nm,
        workspace_low=np.asarray(args.workspace_low, dtype=np.float64),
        workspace_high=np.asarray(args.workspace_high, dtype=np.float64),
    )

    rclpy.init()
    node = SafeFrankaRos2Node(
        namespace=args.namespace,
        controller_name=args.controller_name,
        target_topic=args.target_topic,
        frame_id=args.frame_id,
        limits=limits,
        motion_enabled=args.enable_motion,
    )
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    app = Flask(__name__)

    @app.route("/getstate", methods=["POST"])
    def getstate():
        snap = node.snapshot()
        return jsonify(
            {
                "pose": snap["pose"].tolist(),
                "vel": snap["vel"].tolist(),
                "force": snap["force"].tolist(),
                "torque": snap["torque"].tolist(),
                "q": snap["q"].tolist(),
                "dq": snap["dq"].tolist(),
                "jacobian": snap["jacobian"].reshape(-1).tolist(),
                "gripper_pos": 0,
                "motion_enabled": snap["motion_enabled"],
                "fault": snap["fault"],
            }
        )

    @app.route("/getpos", methods=["POST"])
    def getpos():
        return jsonify(node.snapshot()["pose"].tolist())

    @app.route("/getpos_euler", methods=["POST"])
    def getpos_euler():
        pose = node.snapshot()["pose"]
        euler = Rotation.from_quat(pose[3:]).as_euler("xyz")
        return jsonify(np.concatenate([pose[:3], euler]).tolist())

    @app.route("/getvel", methods=["POST"])
    def getvel():
        return jsonify(node.snapshot()["vel"].tolist())

    @app.route("/getforce", methods=["POST"])
    def getforce():
        return jsonify(node.snapshot()["force"].tolist())

    @app.route("/gettorque", methods=["POST"])
    def gettorque():
        return jsonify(node.snapshot()["torque"].tolist())

    @app.route("/getq", methods=["POST"])
    def getq():
        return jsonify(node.snapshot()["q"].tolist())

    @app.route("/getdq", methods=["POST"])
    def getdq():
        return jsonify(node.snapshot()["dq"].tolist())

    @app.route("/getjacobian", methods=["POST"])
    def getjacobian():
        return jsonify(node.snapshot()["jacobian"].reshape(-1).tolist())

    @app.route("/pose", methods=["POST"])
    def pose():
        payload = request.get_json(force=True, silent=True) or {}
        arr = np.asarray(payload.get("arr", []), dtype=np.float64)
        ok, message = node.validate_and_publish_pose(arr)
        status = 200 if ok else 409
        return jsonify({"success": ok, "message": message}), status

    @app.route("/clearerr", methods=["POST"])
    def clearerr():
        node.clear_fault()
        return jsonify({"success": True, "message": "local fault cleared"})

    @app.route("/update_param", methods=["POST"])
    def update_param():
        return jsonify(
            {
                "success": True,
                "message": "ROS2 safe server ignores dynamic impedance params; configure controller limits instead.",
            }
        )

    @app.route("/startimp", methods=["POST"])
    def startimp():
        return jsonify({"success": False, "message": "Use ros2 control spawner to start controller"}), 409

    @app.route("/stopimp", methods=["POST"])
    def stopimp():
        return jsonify({"success": False, "message": "Use ros2 control to stop controller"}), 409

    @app.route("/jointreset", methods=["POST"])
    def jointreset():
        return jsonify({"success": False, "message": "Joint reset is intentionally not exposed"}), 409

    print("Safe ROS2 Franka HTTP server starting")
    print(f"  namespace: {args.namespace or '<none>'}")
    print(f"  target topic: {node.target_topic}")
    print(f"  motion enabled: {args.enable_motion}")
    print(f"  workspace low/high: {limits.workspace_low} / {limits.workspace_high}")
    print(f"  command delta limits: {limits.max_command_delta_m} m, {limits.max_command_delta_rad} rad")

    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
