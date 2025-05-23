import gymnasium as gym
from gymnasium import spaces
import numpy as np

class RealBotEnv(gym.Env):
    def __init__(self, dataset_path="ppo_training_data.npz"):
        super().__init__()

        # Load real data
        data = np.load(dataset_path)
        self.obs_data = data["obs"]
        self.target_actions = data["actions"]
        self.max_steps = len(self.obs_data)

        # Gym interface
        self.observation_space = spaces.Box(low=-100, high=100, shape=(1,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        self.step_index = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_index = 0
        return self.obs_data[self.step_index].astype(np.float32), {}



    def step(self, action):
        true_action = self.target_actions[self.step_index]

        # Reward: closer to real PPO = better
        reward = -np.square(action - true_action).sum()

        self.step_index += 1
        done = self.step_index >= self.max_steps
        next_obs = (
            self.obs_data[self.step_index].astype(np.float32)
            if not done else np.zeros((1,), dtype=np.float32)
        )
        terminated = done         # episode ended normally
        truncated = False         # we’re not doing time truncation yet
        return next_obs, reward, terminated, truncated, {}


    def render(self, mode="human"):
        pass
