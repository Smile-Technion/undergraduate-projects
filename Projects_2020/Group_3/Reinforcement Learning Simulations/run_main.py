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
env = make_vec_env('ProjectEnv-v1', n_envs=1);

model = DDPG.load("2000Ka")
print("I am done")

# Enjoy trained agent
obs = env.reset()

k=0
sum_rewards=0
while k<10000:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
    k+=1
    sum_rewards+=rewards
    if k%100==0:
        print(sum_rewards)
        sum_rewards=0