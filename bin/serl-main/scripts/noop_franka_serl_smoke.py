#!/usr/bin/env python3
"""Run a SERL-style smoke test without controlling Franka.

This script keeps the SERL observation/action/replay/agent path close to the
real Franka examples, but the environment never sends robot commands. In real
camera mode it still opens the configured D455/D435 RGB streams.
"""

from __future__ import annotations

import argparse
import copy
import os
import sys
import time
from pathlib import Path
from typing import Dict, Tuple

import cv2
import gym
import jax
import numpy as np
import pyrealsense2 as rs

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# SERL stores pretrained ResNet weights under ~/.serl by default. Keep the
# smoke-test cache inside the writable workspace instead.
os.environ["HOME"] = str(REPO_ROOT / ".cache_home")

from experiments.shape_insertion import config as camera_config
from franka_env.camera.rs_capture import RSCapture
from franka_env.camera.video_capture import VideoCapture
from franka_env.envs.relative_env import RelativeFrame
from franka_env.envs.wrappers import Quat2EulerWrapper
from serl_launcher.data.data_store import MemoryEfficientReplayBufferDataStore
from serl_launcher.utils.launcher import make_drq_agent
from serl_launcher.wrappers.chunking import ChunkingWrapper
from serl_launcher.wrappers.serl_obs_wrappers import SERLObsWrapper


Crop = Tuple[int, int, int, int]


def list_realsense_serials() -> list[str]:
    return [d.get_info(rs.camera_info.serial_number) for d in rs.context().devices]


def safe_crop(img: np.ndarray, crop: Crop) -> np.ndarray:
    h, w = img.shape[:2]
    x1, y1, x2, y2 = crop
    x1 = max(0, int(x1))
    y1 = max(0, int(y1))
    x2 = min(w, int(x2))
    y2 = min(h, int(y2))
    if x1 >= x2 or y1 >= y2:
        raise ValueError(f"Invalid crop {crop} for image shape {(w, h)}")
    return img[y1:y2, x1:x2]


class RealDualCameraReader:
    def __init__(self, image_size: int):
        self.image_size = image_size
        serials = {
            "d455": camera_config.D455_SERIAL,
            "d435": camera_config.D435_SERIAL,
        }
        missing = [name for name, serial in serials.items() if not serial]
        if missing:
            available = list_realsense_serials()
            raise RuntimeError(
                "Missing camera serial(s) in experiments/shape_insertion/config.py: "
                f"{missing}. Available RealSense serials: {available}"
            )

        available = list_realsense_serials()
        not_found = [serial for serial in serials.values() if serial not in available]
        if not_found:
            raise RuntimeError(
                f"Configured RealSense serial(s) not found: {not_found}. "
                f"Available: {available}"
            )

        self.crops = {
            "d455": camera_config.D455_CROP,
            "d435": camera_config.D435_CROP,
        }
        self.caps = {
            name: VideoCapture(
                RSCapture(
                    name=name,
                    serial_number=serial,
                    dim=(camera_config.WIDTH, camera_config.HEIGHT),
                    fps=camera_config.FPS,
                    depth=False,
                )
            )
            for name, serial in serials.items()
        }

    def read(self) -> Dict[str, np.ndarray]:
        images = {}
        for name, cap in self.caps.items():
            bgr = cap.read()
            cropped = safe_crop(bgr, self.crops[name])
            resized = cv2.resize(cropped, (self.image_size, self.image_size))
            images[name] = resized[..., ::-1]
        return images

    def close(self) -> None:
        for cap in self.caps.values():
            cap.close()


class DummyDualCameraReader:
    def __init__(self, image_size: int):
        self.image_size = image_size
        self.t = 0

    def read(self) -> Dict[str, np.ndarray]:
        self.t += 1
        img = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        cv2.circle(
            img,
            (self.image_size // 2, self.image_size // 2),
            18 + (self.t % 12),
            (230, 45, 130),
            -1,
        )
        cv2.rectangle(img, (34, 34), (94, 94), (20, 20, 20), 3)
        return {"d455": img.copy(), "d435": np.flipud(img).copy()}

    def close(self) -> None:
        return None


class NoOpFrankaCameraEnv(gym.Env):
    """SERL-shaped Franka env that reads cameras but never controls the robot."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        camera_mode: str,
        image_size: int = 128,
        max_episode_length: int = 10,
        hz: float = 10.0,
        preview: bool = False,
    ):
        self.image_size = image_size
        self.max_episode_length = max_episode_length
        self.hz = hz
        self.preview = preview
        self.path_length = 0

        self.reader = (
            RealDualCameraReader(image_size)
            if camera_mode == "real"
            else DummyDualCameraReader(image_size)
        )

        self.action_space = gym.spaces.Box(
            low=-np.ones((6,), dtype=np.float32),
            high=np.ones((6,), dtype=np.float32),
            dtype=np.float32,
        )
        self.observation_space = gym.spaces.Dict(
            {
                "state": gym.spaces.Dict(
                    {
                        "tcp_pose": gym.spaces.Box(-np.inf, np.inf, shape=(7,)),
                        "tcp_vel": gym.spaces.Box(-np.inf, np.inf, shape=(6,)),
                        "gripper_pose": gym.spaces.Box(-1, 1, shape=(1,)),
                        "tcp_force": gym.spaces.Box(-np.inf, np.inf, shape=(3,)),
                        "tcp_torque": gym.spaces.Box(-np.inf, np.inf, shape=(3,)),
                    }
                ),
                "images": gym.spaces.Dict(
                    {
                        "d455": gym.spaces.Box(
                            0,
                            255,
                            shape=(image_size, image_size, 3),
                            dtype=np.uint8,
                        ),
                        "d435": gym.spaces.Box(
                            0,
                            255,
                            shape=(image_size, image_size, 3),
                            dtype=np.uint8,
                        ),
                    }
                ),
            }
        )

        self.static_pose = np.array([0.55, 0.0, 0.20, 0.0, 0.0, 0.0, 1.0])

    def _get_obs(self) -> dict:
        images = self.reader.read()
        if self.preview:
            cv2.imshow("noop_serl_d455", images["d455"][..., ::-1])
            cv2.imshow("noop_serl_d435", images["d435"][..., ::-1])
            cv2.waitKey(1)

        return copy.deepcopy(
            {
                "state": {
                    "tcp_pose": self.static_pose.copy(),
                    "tcp_vel": np.zeros((6,), dtype=np.float64),
                    "gripper_pose": np.zeros((1,), dtype=np.float64),
                    "tcp_force": np.zeros((3,), dtype=np.float64),
                    "tcp_torque": np.zeros((3,), dtype=np.float64),
                },
                "images": images,
            }
        )

    def reset(self, **kwargs):
        self.path_length = 0
        return self._get_obs(), {}

    def step(self, action):
        del action
        start = time.time()
        self.path_length += 1
        obs = self._get_obs()
        reward = np.float32(0.0)
        done = self.path_length >= self.max_episode_length
        elapsed = time.time() - start
        time.sleep(max(0.0, (1.0 / self.hz) - elapsed))
        return obs, reward, done, False, {"noop_robot_control": True}

    def close(self):
        self.reader.close()
        if self.preview:
            cv2.destroyAllWindows()


def build_env(args: argparse.Namespace) -> gym.Env:
    env = NoOpFrankaCameraEnv(
        camera_mode=args.camera_mode,
        image_size=args.image_size,
        max_episode_length=args.episode_length,
        hz=args.hz,
        preview=args.preview,
    )
    env = RelativeFrame(env)
    env = Quat2EulerWrapper(env)
    env = SERLObsWrapper(env)
    env = ChunkingWrapper(env, obs_horizon=1, act_exec_horizon=None)
    return env


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SERL smoke test with real cameras and no Franka control"
    )
    parser.add_argument("--camera-mode", choices=("real", "dummy"), default="real")
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--episode-length", type=int, default=10)
    parser.add_argument("--image-size", type=int, default=128)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--encoder-type", default="resnet-pretrained")
    parser.add_argument(
        "--skip-agent",
        action="store_true",
        help="Only test env/wrappers/cameras/replay buffer, without DrQ init.",
    )
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--list-cameras", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.list_cameras:
        print("Available RealSense serials:", list_realsense_serials())
        return

    env = build_env(args)
    image_keys = [key for key in env.observation_space.keys() if key != "state"]
    print("Observation space:", env.observation_space)
    print("Action space:", env.action_space)
    print("Image keys:", image_keys)

    replay_buffer = MemoryEfficientReplayBufferDataStore(
        env.observation_space,
        env.action_space,
        capacity=max(100, args.steps + 1),
        image_keys=image_keys,
    )
    agent = None
    if not args.skip_agent:
        agent = make_drq_agent(
            seed=0,
            sample_obs=env.observation_space.sample(),
            sample_action=env.action_space.sample(),
            image_keys=image_keys,
            encoder_type=args.encoder_type,
        )

    rng = jax.random.PRNGKey(0)
    obs, _ = env.reset()
    inserted = 0
    try:
        for step in range(args.steps):
            if agent is None:
                action = env.action_space.sample()
            else:
                rng, key = jax.random.split(rng)
                action = agent.sample_actions(
                    observations=jax.device_put(obs),
                    seed=key,
                    deterministic=False,
                )
                action = np.asarray(jax.device_get(action))
            next_obs, reward, done, truncated, info = env.step(action)
            transition = {
                "observations": obs,
                "actions": action,
                "next_observations": next_obs,
                "rewards": np.asarray(reward, dtype=np.float32),
                "masks": np.float32(1.0 - done),
                "dones": done,
            }
            replay_buffer.insert(transition)
            inserted += 1
            obs = next_obs
            print(
                f"step={step + 1} reward={float(reward):.1f} "
                f"done={done} noop={info['noop_robot_control']}"
            )
            if done or truncated:
                obs, _ = env.reset()
    finally:
        env.close()

    print(f"Smoke test complete. Replay buffer size: {len(replay_buffer)}")
    if inserted != args.steps:
        raise RuntimeError(f"Expected {args.steps} inserts, got {inserted}")


if __name__ == "__main__":
    main()
