import gym
import training.gym_spotmicroai
from simulation import example_automatic_gait
env = gym.make('spotmicroai-v0')
while True:
    env.step(env.action_space)
    env.render()
