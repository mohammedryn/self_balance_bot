import pandas as pd
import numpy as np

# Load PPO log
df = pd.read_csv("ppo_log.csv")

# Parse action into two values
df['ppo_0'] = df['ppo_action'].apply(lambda x: float(x.strip('[]').split()[0]))
df['ppo_1'] = df['ppo_action'].apply(lambda x: float(x.strip('[]').split()[1]))
df['ppo_avg'] = (df['ppo_0'] + df['ppo_1']) / 2

# Define observation space (for now: just 'diff', you can expand later)
obs = df['diff'].values.reshape(-1, 1)

# Define target actions
actions = df['ppo_avg'].values.reshape(-1, 1)

# Save data to file
np.savez("ppo_training_data.npz", obs=obs, actions=actions)
print("Saved obs and actions to ppo_training_data.npz")
