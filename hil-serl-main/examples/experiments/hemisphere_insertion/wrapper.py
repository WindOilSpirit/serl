import copy

import gymnasium as gym
import numpy as np


class HemisphereInsertionDummyEnv(gym.Env):
    """Minimal no-hardware env for validating the experiment entrypoint."""

    metadata = {"render_modes": []}

    def __init__(self, config):
        self.config = config
        self.max_episode_steps = config.MAX_EPISODE_STEPS
        self.curr_path_length = 0
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
        return self._get_obs(), {"succeed": False}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.curr_path_length += 1
        obs = self._get_obs()
        done = self.curr_path_length >= self.max_episode_steps
        info = {
            "succeed": False,
            "dry_run": True,
            "action": copy.deepcopy(action),
        }
        return obs, 0, done, False, info

    def _get_obs(self):
        state = {
            "tcp_pose": np.zeros((7,), dtype=np.float32),
            "tcp_vel": np.zeros((6,), dtype=np.float32),
            "joint_positions": np.zeros((7,), dtype=np.float32),
        }
        images = {
            "image_d455": np.zeros(
                (self.config.IMAGE_SIZE, self.config.IMAGE_SIZE, 3), dtype=np.uint8
            )
        }
        return {"state": state, "images": images}


class HemisphereInsertionEnv(HemisphereInsertionDummyEnv):
    """Placeholder for the future ROS2-backed Franka env."""

    def __init__(self, config):
        super().__init__(config)


class HemisphereInsertionDemoWrapper(gym.Wrapper):
    """End episodes by horizon, optionally asking whether to save demos."""

    def __init__(self, env, max_episode_steps, ask_on_done=True):
        super().__init__(env)
        self.max_episode_steps = max_episode_steps
        self.ask_on_done = ask_on_done
        self.episode_steps = 0

    def reset(self, **kwargs):
        self.episode_steps = 0
        obs, info = self.env.reset(**kwargs)
        info["succeed"] = False
        return obs, info

    def step(self, action):
        obs, reward, done, truncated, info = self.env.step(action)
        self.episode_steps += 1
        horizon_done = self.episode_steps >= self.max_episode_steps
        done = bool(done or horizon_done)
        if done and self.ask_on_done:
            info["succeed"] = self._ask_save_demo()
            reward = int(info["succeed"])
        else:
            info["succeed"] = False
        info["episode_steps"] = self.episode_steps
        info["horizon_done"] = horizon_done
        return obs, reward, done, truncated, info

    def _ask_save_demo(self):
        while True:
            answer = input("Save this hemisphere demo? [y/n]: ").strip().lower()
            if answer in ("y", "yes", "1"):
                return True
            if answer in ("n", "no", "0"):
                return False
            print("Please answer y or n.")
