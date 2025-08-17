import gymnasium as gym

# Create the training environment itself
env = gym.make("CartPole-v1", render_mode="human")

# Reset environment, creating a new episode
observation, info = env.reset()

# State Starting Location
print(f"Starting Observation: {observation}")

episode_over = False
tot_reward = 0

while not episode_over:
    # Choose an action: cart moving left = 0, moving right = 1
    action = env.action_space.sample()

    # Take the action and see what happens
    observation, reward, terminated, truncated, info = env.step(action)
    tot_reward += reward
    episode_ver = terminated or truncated


print("Episode Finished, total reward: {tot_reward}")
env.close()
