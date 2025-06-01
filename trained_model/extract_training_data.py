import pandas as pd
import numpy as np

# Load PPO log
df = pd.read_csv("ppo_log.csv")

# Extract PPO scalar action
df['ppo_scalar'] = df['ppo_action'].apply(lambda x: float(x.strip('[]').split()[0]))

# Use ERROR as the observation (single-dimensional input)
obs = df['error'].values.reshape(-1, 1)

# Use PPO scalar action as target
actions = df['ppo_scalar'].values.reshape(-1, 1)

# Save data
np.savez("ppo_training_data.npz", obs=obs, actions=actions)
print("Saved obs and actions to ppo_training_data.npz")
