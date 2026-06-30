import time

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


class ROS2FrankaEnv(gym.Env):
    """Read-only ROS2 Franka observation env.

    By default this env does not publish robot commands. Set publish_actions=True
    explicitly for the low-risk 1 mm action bridge test.
    """

    metadata = {"render_modes": []}

    def __init__(self, config, publish_actions=False):
        self.config = config
        self.publish_actions = bool(publish_actions)
        self.max_episode_steps = config.MAX_EPISODE_STEPS
        self.curr_path_length = 0
        self._latest_image = None
        self._latest_pose = None
        self._latest_joint_positions = None
        self._owns_rclpy = False

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        self.node = Node("hemisphere_insertion_readonly_env")
        self.node.create_subscription(
            Image, config.RGB_TOPIC, self._image_cb, qos_profile_sensor_data
        )
        self.node.create_subscription(
            PoseStamped, config.TCP_POSE_TOPIC, self._pose_cb, qos_profile_sensor_data
        )
        self.node.create_subscription(
            JointState, config.JOINT_STATES_TOPIC, self._joint_cb, qos_profile_sensor_data
        )
        self.target_pub = self.node.create_publisher(PoseStamped, config.TARGET_POSE_TOPIC, 10)

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(config.ACTION_DIM,),
            dtype=np.float32,
        )
        self.observation_space = gym.spaces.Dict(
            {
                "state": gym.spaces.Dict(
                    {
                        "tcp_pose": gym.spaces.Box(
                            -np.inf, np.inf, shape=(7,), dtype=np.float32
                        ),
                        "tcp_vel": gym.spaces.Box(
                            -np.inf, np.inf, shape=(6,), dtype=np.float32
                        ),
                        "joint_positions": gym.spaces.Box(
                            -np.inf, np.inf, shape=(7,), dtype=np.float32
                        ),
                    }
                ),
                "images": gym.spaces.Dict(
                    {
                        "image_d455": gym.spaces.Box(
                            0,
                            255,
                            shape=(config.IMAGE_SIZE, config.IMAGE_SIZE, 3),
                            dtype=np.uint8,
                        )
                    }
                ),
            }
        )

    def reset(self, **kwargs):
        super().reset(seed=kwargs.get("seed"))
        self.curr_path_length = 0
        self._wait_for_observation()
        return self._get_obs(), {"succeed": False, "readonly": not self.publish_actions}

    def step(self, action):
        start_time = time.time()
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.curr_path_length += 1
        self._wait_for_observation()
        target_pose, applied_action = self._delta_to_target_pose(action)
        action_published = False
        if self.publish_actions:
            self._publish_target_pose(target_pose)
            action_published = True
            rclpy.spin_once(self.node, timeout_sec=0.0)
        self._wait_for_observation()
        info = {
            "succeed": False,
            "readonly": not self.publish_actions,
            "action_published": action_published,
            "applied_action": applied_action.astype(np.float32),
            "target_pose": target_pose.astype(np.float32),
        }
        elapsed = time.time() - start_time
        if elapsed < self.config.STEP_SLEEP_SEC:
            time.sleep(self.config.STEP_SLEEP_SEC - elapsed)
            self._wait_for_observation()
        return self._get_obs(), 0, False, False, info

    def _delta_to_target_pose(self, action):
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (6,):
            raise ValueError(f"Expected 6D action, got shape {action.shape}")

        current_pose = self._latest_pose.astype(np.float64).copy()
        translation = np.clip(action[:3], -1.0, 1.0) * self.config.ACTION_TRANSLATION_SCALE
        translation = np.clip(
            translation,
            -self.config.MAX_TRANSLATION_STEP,
            self.config.MAX_TRANSLATION_STEP,
        )
        if np.linalg.norm(translation, ord=np.inf) > 0.002:
            raise ValueError(f"Translation step exceeds 2 mm safety limit: {translation}")

        rotation = np.clip(action[3:6], -1.0, 1.0) * self.config.ACTION_ROTATION_SCALE
        rotation = np.clip(
            rotation,
            -self.config.MAX_ROTATION_STEP,
            self.config.MAX_ROTATION_STEP,
        )

        target_pose = current_pose.copy()
        target_pose[:3] = current_pose[:3] + translation
        low = np.asarray(self.config.WORKSPACE_LOW, dtype=np.float64)
        high = np.asarray(self.config.WORKSPACE_HIGH, dtype=np.float64)
        clipped_position = np.clip(target_pose[:3], low, high)
        if not np.allclose(clipped_position, target_pose[:3]):
            raise ValueError(
                f"Target pose outside workspace: target={target_pose[:3]} "
                f"low={low} high={high}"
            )
        target_pose[:3] = clipped_position
        target_pose[3:7] = self._normalize_quat(current_pose[3:7])
        applied_action = np.zeros((6,), dtype=np.float64)
        applied_action[:3] = translation
        applied_action[3:6] = rotation
        return target_pose, applied_action

    def _publish_target_pose(self, target_pose):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.config.FRAME_ID
        msg.pose.position.x = float(target_pose[0])
        msg.pose.position.y = float(target_pose[1])
        msg.pose.position.z = float(target_pose[2])
        quat = self._normalize_quat(target_pose[3:7])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self.target_pub.publish(msg)

    def _normalize_quat(self, quat):
        quat = np.asarray(quat, dtype=np.float64)
        norm = float(np.linalg.norm(quat))
        if not np.isfinite(norm) or norm < 1.0e-9:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        return quat / norm

    def _wait_for_observation(self):
        deadline = time.time() + self.config.OBS_TIMEOUT_SEC
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self._latest_image is not None
                and self._latest_pose is not None
                and self._latest_joint_positions is not None
            ):
                return
        missing = []
        if self._latest_image is None:
            missing.append(self.config.RGB_TOPIC)
        if self._latest_pose is None:
            missing.append(self.config.TCP_POSE_TOPIC)
        if self._latest_joint_positions is None:
            missing.append(self.config.JOINT_STATES_TOPIC)
        raise TimeoutError(f"Timed out waiting for ROS2 observation topics: {missing}")

    def _get_obs(self):
        state = {
            "tcp_pose": self._latest_pose.astype(np.float32),
            "tcp_vel": np.zeros((6,), dtype=np.float32),
            "joint_positions": self._latest_joint_positions.astype(np.float32),
        }
        images = {"image_d455": self._latest_image.copy()}
        return {"state": state, "images": images}

    def _image_cb(self, msg):
        image = self._image_to_numpy(msg)
        self._latest_image = self._resize_nearest(image, self.config.IMAGE_SIZE)

    def _pose_cb(self, msg):
        pose = msg.pose
        self._latest_pose = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float32,
        )

    def _joint_cb(self, msg):
        positions = np.asarray(msg.position[:7], dtype=np.float32)
        if positions.shape[0] < 7:
            padded = np.zeros((7,), dtype=np.float32)
            padded[: positions.shape[0]] = positions
            positions = padded
        self._latest_joint_positions = positions

    def _image_to_numpy(self, msg):
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        encoding = msg.encoding.lower()
        if encoding not in channels_by_encoding:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        channels = channels_by_encoding[encoding]
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, channels
        )
        if encoding == "bgr8":
            arr = arr[..., ::-1]
        elif encoding == "bgra8":
            arr = arr[..., [2, 1, 0]]
        elif encoding == "rgba8":
            arr = arr[..., :3]
        elif encoding == "mono8":
            arr = np.repeat(arr, 3, axis=2)
        return np.ascontiguousarray(arr)

    def _resize_nearest(self, image, size):
        if image.shape[:2] == (size, size):
            return image
        y_idx = np.linspace(0, image.shape[0] - 1, size).astype(np.int64)
        x_idx = np.linspace(0, image.shape[1] - 1, size).astype(np.int64)
        return image[y_idx][:, x_idx].astype(np.uint8, copy=False)

    def close(self):
        if hasattr(self, "node"):
            self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
