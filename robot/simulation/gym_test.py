import gym
import gym_spotmicroai
env = gym.make('spotmicroai-v0')
while True:
    env.step(env.action_space)
    env.render()
