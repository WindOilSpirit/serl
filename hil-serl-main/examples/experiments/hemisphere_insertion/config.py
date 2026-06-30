import os
from dataclasses import dataclass

import jax
import jax.numpy as jnp

from experiments.config import DefaultTrainingConfig
from experiments.hemisphere_insertion.wrapper import (
    HemisphereInsertionDemoWrapper,
    HemisphereInsertionDummyEnv,
)
from franka_env.envs.wrappers import (
    MultiCameraBinaryRewardClassifierWrapper,
    SpacemouseIntervention,
)
from serl_launcher.wrappers.chunking import ChunkingWrapper
from serl_launcher.networks.reward_classifier import load_classifier_func
from serl_launcher.wrappers.serl_obs_wrappers import SERLObsWrapper


@dataclass
class EnvConfig:
    ACTION_DIM: int = 6
    IMAGE_SIZE: int = 128
    MAX_EPISODE_STEPS: int = 50
    RGB_TOPIC: str = "/camera/d435/color/image_raw"
    TCP_POSE_TOPIC: str = "/franka_robot_state_broadcaster/current_pose"
    JOINT_STATES_TOPIC: str = "/joint_states"
    TARGET_POSE_TOPIC: str = "/serl_cartesian_impedance_controller/target_pose"
    FRAME_ID: str = "base"
    OBS_TIMEOUT_SEC: float = 5.0
    STEP_SLEEP_SEC: float = 0.1
    ACTION_TRANSLATION_SCALE: float = 1.0
    ACTION_ROTATION_SCALE: float = 0.0
    MAX_TRANSLATION_STEP: float = 0.001
    MAX_ROTATION_STEP: float = 0.0
    WORKSPACE_LOW: tuple = (0.25, -0.20, 0.04)
    WORKSPACE_HIGH: tuple = (0.75, 0.25, 0.75)


class TrainConfig(DefaultTrainingConfig):
    image_keys = ["image_d455"]
    classifier_keys = ["image_d455"]
    proprio_keys = ["tcp_pose", "tcp_vel", "joint_positions"]
    record_demos_classifier = False
    record_demos_publish_actions = True
    record_success_fail_ask_on_done = False
    record_success_fail_publish_actions = True
    max_traj_length = EnvConfig.MAX_EPISODE_STEPS
    max_episode_steps = EnvConfig.MAX_EPISODE_STEPS
    action_dim = EnvConfig.ACTION_DIM
    encoder_type = "resnet-pretrained"
    setup_mode = "single-arm-fixed-gripper"

    def get_environment(
        self,
        fake_env=False,
        save_video=False,
        classifier=False,
        publish_actions=False,
        ask_on_done=True,
    ):
        env_config = EnvConfig()
        if fake_env:
            env = HemisphereInsertionDummyEnv(config=env_config)
        else:
            from franka_env.envs.ros2_franka_env import ROS2FrankaEnv

            env = ROS2FrankaEnv(config=env_config, publish_actions=publish_actions)
            env = SpacemouseIntervention(env)
        env = HemisphereInsertionDemoWrapper(
            env,
            max_episode_steps=env_config.MAX_EPISODE_STEPS,
            ask_on_done=ask_on_done,
        )
        env = SERLObsWrapper(env, proprio_keys=self.proprio_keys)
        env = ChunkingWrapper(env, obs_horizon=1, act_exec_horizon=None)
        if classifier and self.classifier_keys is not None:
            classifier_func = load_classifier_func(
                key=jax.random.PRNGKey(0),
                sample=env.observation_space.sample(),
                image_keys=self.classifier_keys,
                checkpoint_path=os.path.abspath("classifier_ckpt/"),
            )

            def reward_func(obs):
                sigmoid = lambda x: 1 / (1 + jnp.exp(-x))
                return int(sigmoid(classifier_func(obs)) > 0.7)

            env = MultiCameraBinaryRewardClassifierWrapper(env, reward_func)
        return env

    def process_demos(self, demo):
        return demo
