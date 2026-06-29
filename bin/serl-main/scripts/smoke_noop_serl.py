#!/usr/bin/env python3
"""Smoke test the SERL stack without commanding a Franka robot.

This script keeps the SERL-facing API close to the real Franka env, but all
robot control is a no-op. It does not import or start ROS.
"""

from __future__ import annotations

import argparse
import copy

import gym
import jax
import numpy as np

from franka_env.envs.relative_env import RelativeFrame
from franka_env.envs.wrappers import GripperCloseEnv, Quat2EulerWrapper
from franka_env.utils.rotations import euler_2_quat
from serl_launcher.data.data_store import MemoryEfficientReplayBufferDataStore
from serl_launcher.utils.launcher import make_drq_agent
from serl_launcher.wrappers.chunking import ChunkingWrapper
from serl_launcher.wrappers.serl_obs_wrappers import SERLObsWrapper


class NoOpFrankaEnv(gym.Env):
    """A Franka-shaped env that never sends robot commands."""

    metadata = {"render_modes": []}

    def __init__(self, max_episode_length: int = 20, image_size: int = 128):
        super().__init__()
        self.max_episode_length = max_episode_length
        self.image_size = image_size
        self.path_length = 0
        self.reset_pose = np.array(
            [0.55, 0.0, 0.18, *euler_2_quat(np.array([np.pi, 0.0, 0.0]))],
            dtype=np.float32,
        )
        self.curr_pose = self.reset_pose.copy()

        self.action_space = gym.spaces.Box(
            low=-np.ones((7,), dtype=np.float32),
            high=np.ones((7,), dtype=np.float32),
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
                        "gripper_pose": gym.spaces.Box(
                            -1.0, 1.0, shape=(1,), dtype=np.float32
                        ),
                        "tcp_force": gym.spaces.Box(
                            -np.inf, np.inf, shape=(3,), dtype=np.float32
                        ),
                        "tcp_torque": gym.spaces.Box(
                            -np.inf, np.inf, shape=(3,), dtype=np.float32
                        ),
                    }
                ),
                "images": gym.spaces.Dict(
                    {
                        "wrist_1": gym.spaces.Box(
                            0,
                            255,
                            shape=(image_size, image_size, 3),
                            dtype=np.uint8,
                        ),
                        "wrist_2": gym.spaces.Box(
                            0,
                            255,
                            shape=(image_size, image_size, 3),
                            dtype=np.uint8,
                        ),
                    }
                ),
            }
        )

    def reset(self, **kwargs):
        self.path_length = 0
        self.curr_pose = self.reset_pose.copy()
        return self._get_obs(), {}

    def step(self, action):
        del action
        self.path_length += 1
        done = self.path_length >= self.max_episode_length
        reward = np.float32(0.0)
        return self._get_obs(), reward, done, False, {"noop_control": True}

    def _get_obs(self):
        image = self._synthetic_image()
        return copy.deepcopy(
            {
                "state": {
                    "tcp_pose": self.curr_pose.astype(np.float32),
                    "tcp_vel": np.zeros((6,), dtype=np.float32),
                    "gripper_pose": np.zeros((1,), dtype=np.float32),
                    "tcp_force": np.zeros((3,), dtype=np.float32),
                    "tcp_torque": np.zeros((3,), dtype=np.float32),
                },
                "images": {
                    "wrist_1": image,
                    "wrist_2": np.flip(image, axis=1).copy(),
                },
            }
        )

    def _synthetic_image(self):
        img = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        yy, xx = np.ogrid[: self.image_size, : self.image_size]
        center = self.image_size // 2
        red_mask = (xx - center + 18) ** 2 + (yy - center) ** 2 < 18**2
        dark_mask = (xx - center - 18) ** 2 + (yy - center) ** 2 < 22**2
        img[..., 1] = 40
        img[dark_mask] = np.array([10, 10, 10], dtype=np.uint8)
        img[red_mask] = np.array([220, 40, 110], dtype=np.uint8)
        return img


def build_env(max_episode_length: int, image_size: int):
    env = NoOpFrankaEnv(max_episode_length=max_episode_length, image_size=image_size)
    env = GripperCloseEnv(env)
    env = RelativeFrame(env)
    env = Quat2EulerWrapper(env)
    env = SERLObsWrapper(env)
    env = ChunkingWrapper(env, obs_horizon=1, act_exec_horizon=None)
    return env


def main():
    parser = argparse.ArgumentParser(description="Run a no-op SERL smoke test")
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--image-size", type=int, default=128)
    parser.add_argument("--encoder-type", default="small")
    args = parser.parse_args()

    env = build_env(max_episode_length=args.steps + 1, image_size=args.image_size)
    obs, _ = env.reset()
    image_keys = [key for key in env.observation_space.keys() if key != "state"]

    agent = make_drq_agent(
        seed=0,
        sample_obs=env.observation_space.sample(),
        sample_action=env.action_space.sample(),
        image_keys=image_keys,
        encoder_type=args.encoder_type,
    )
    replay_buffer = MemoryEfficientReplayBufferDataStore(
        env.observation_space,
        env.action_space,
        capacity=max(10, args.steps + 1),
        image_keys=image_keys,
    )

    rng = jax.random.PRNGKey(0)
    for step in range(args.steps):
        rng, key = jax.random.split(rng)
        action = np.asarray(
            jax.device_get(
                agent.sample_actions(
                    observations=jax.device_put(obs),
                    seed=key,
                    deterministic=False,
                )
            )
        )
        next_obs, reward, done, truncated, info = env.step(action)
        transition = {
            "observations": obs,
            "actions": action,
            "next_observations": next_obs,
            "rewards": np.asarray(reward, dtype=np.float32),
            "masks": np.asarray(1.0 - done, dtype=np.float32),
            "dones": np.asarray(done, dtype=bool),
        }
        replay_buffer.insert(transition)
        obs = next_obs
        print(
            f"step={step + 1} action_shape={action.shape} "
            f"reward={float(reward):.1f} done={done} noop={info['noop_control']}"
        )
        if done or truncated:
            obs, _ = env.reset()

    batch = replay_buffer.sample(batch_size=min(args.steps, len(replay_buffer)))
    print("smoke test ok")
    print(f"image_keys={image_keys}")
    print(f"replay_size={len(replay_buffer)}")
    print(f"sampled_state_shape={batch['observations']['state'].shape}")


if __name__ == "__main__":
    main()
