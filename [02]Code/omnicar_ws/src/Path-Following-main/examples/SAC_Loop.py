from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
import numpy as np
import yaml
from matplotlib import animation, pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from torch.backends.mkl import verbose

from RL.Environment import *
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import dummy_vec_env, DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from gymnasium.wrappers import FlattenObservation

from plotting.plot_utils import animate_cbf_robot

with open("../config/config.yaml", "rb") as f:
    conf = yaml.safe_load(f.read())

train_env = (FlattenObservation(Environment(conf)))
test_env = (FlattenObservation(Environment(conf)))

eval_callback = EvalCallback(
    test_env,
    n_eval_episodes=1,
    best_model_save_path="./logs/best_model/",
    log_path="./logs/results",
    eval_freq=10000,
    deterministic=True
)

#policy_kwargs = dict(net_arch=[256, 256])  # Slightly larger network

model = SAC(
    "MlpPolicy",
    train_env,
    train_freq=1,
    learning_starts=10000,
    gamma=0.995,
    tensorboard_log="./logs/sac_tensorboard/",
)

model.learn(total_timesteps=20_000_000, callback=eval_callback,progress_bar=True)
model.save("sac_env.zip")


model = SAC.load("sac_env.zip", env=test_env)


x_log, y_log, cbf_log, obs_log, theta_log = [], [], [], [], []
v_log, delta_log, vd_log = [], [], []
for i in range(1):
    terminated =  False
    truncated = False
    obs, info = test_env.reset()
    while not (terminated or truncated):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = test_env.step(action)
        print(test_env.env.path.gamma,test_env.env.t,action,test_env.env.vd)
        # Log for visualization
        x_log.append(info["state"][0])
        y_log.append(info["state"][1])
        cbf_log.append(info["cbf"])
        obs_log.append(obs)
        theta_log.append(info["state"][2])
        v_log.append(info["state"][3])
        delta_log.append(info["state"][4])
        vd_log.append(test_env.env.vd)







animate_cbf_robot(x_log, y_log, theta_log, cbf_log, v_log, delta_log, vd_log)
