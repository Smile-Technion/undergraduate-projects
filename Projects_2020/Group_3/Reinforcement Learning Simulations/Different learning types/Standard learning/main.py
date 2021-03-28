##
import gym
import numpy as np
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common import make_vec_env
from stable_baselines import PPO2
from stable_baselines import DQN
from stable_baselines import A2C
from stable_baselines import DDPG
from stable_baselines import TRPO

from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec

#import ProjectEnv

# multiprocess environment
env = make_vec_env('ProjectEnv-v1', n_envs=1)
#env = make_vec_env('Reacher-v2', n_envs=1)
# the noise objects for DDPG
n_actions = env.action_space.shape[-1]
param_noise = None
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.2) * np.ones(n_actions))

model = DDPG(MlpPolicy, env, verbose=1, param_noise=param_noise, action_noise=action_noise)
model.learn(total_timesteps=1000)
model.save("2000Ka")

del model # remove to demonstrate saving and loading

model = DDPG.load("2000Ka")
print("I am done")

# Enjoy trained agent
obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()