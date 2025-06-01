from stable_baselines3 import PPO

# Load the model
model = PPO.load("BaseConfig_PPO_AW_01.zip")

# Dummy observation (replace later with real angle/gyro diff)
obs = [0.0] * 16

# Run inference
action, _states = model.predict(obs, deterministic=True)

print(f"PPO model output: {action}")
