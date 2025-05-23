import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV
df = pd.read_csv("ppo_log.csv")

# Convert timestamp to relative time
df['time'] = df['time'] - df['time'].iloc[0]

# Parse PPO action string into two separate values
df['ppo_0'] = df['ppo_action'].apply(lambda x: float(x.strip('[]').split()[0]))
df['ppo_1'] = df['ppo_action'].apply(lambda x: float(x.strip('[]').split()[1]))

# === Plot 1: Angle vs Target ===
plt.figure()
plt.plot(df['time'], df['angle'], label='Angle')
plt.plot(df['time'], df['target'], label='Target')
plt.plot(df['time'], df['diff'], label='Diff')
plt.title("Angle vs Target Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Angle (°)")
plt.legend()

# === Plot 2: PPO Adjustment ===
plt.figure()
plt.plot(df['time'], df['adjustment'], label='PPO Adjustment')
plt.title("PPO Output Adjustment Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Adjustment")
plt.legend()

# === Plot 3: PPO Action Components ===
plt.figure()
plt.plot(df['time'], df['ppo_0'], label='PPO Action 0')
plt.plot(df['time'], df['ppo_1'], label='PPO Action 1')
plt.title("Raw PPO Actions Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Action Value")
plt.legend()

plt.show()
