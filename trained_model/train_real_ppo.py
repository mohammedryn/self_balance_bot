from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from realbot_env import RealBotEnv

# Create environment
env = RealBotEnv()
check_env(env, warn=True)

# Define PPO model
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=0.0003,
    gamma=0.99,
    n_steps=64,
    batch_size=32,
    n_epochs=10
)

# Train the model
model.learn(total_timesteps=20000)

# Save the model
model.save("ppo_realbot_trained.zip")
print("Training complete. Model saved as ppo_realbot_trained.zip")
