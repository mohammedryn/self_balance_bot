from realbot_env import RealBotEnv
env = RealBotEnv()

obs = env.reset()
total_reward = 0

for _ in range(100):
    action = env.action_space.sample()
    obs, reward, done, _ = env.step(action)
    total_reward += reward
    if done:
        break

print("Test run finished. Total reward:", total_reward)
