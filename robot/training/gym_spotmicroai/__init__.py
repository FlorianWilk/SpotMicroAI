from gym.envs.registration import register

register(
    id='spotmicroai-v0',
    entry_point='gym_spotmicroai.envs:SpotMicroAIEnv',
)
