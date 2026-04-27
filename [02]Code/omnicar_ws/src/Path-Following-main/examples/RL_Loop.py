import numpy as np
import yaml
from matplotlib import animation, pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback

from RL.Environment_ROM import *
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import dummy_vec_env, DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from gymnasium.wrappers import FlattenObservation

from plotting.plot_utils import  animate_cbf_robot

with open("../config/config_RL.yaml", "rb") as f:
    conf = yaml.safe_load(f.read())





def make_env():
    return FlattenObservation(Environment(conf))
if __name__=="__main__":
    env = SubprocVecEnv([make_env for _ in range(64)])
    test_env = FlattenObservation(Environment(conf))
    #check_env(env)
    #env.reset()

    eval_callback = EvalCallback(test_env,n_eval_episodes=1,  best_model_save_path="./logs/best_model/", log_path="./logs/results", eval_freq=100000 // 64, deterministic=True)
    #100000
    policy_kwargs = dict(
        net_arch=[256,256]
    )

    model = PPO("MlpPolicy", env, gamma=0.995,policy_kwargs=policy_kwargs,verbose=0,tensorboard_log="./logs/ROM_tensorboard/")
    #model = PPO.load("logs/best_model/new_goat.zip",env=env,gamma=0.995,tensorboard_log="./logs/sac_tensorboard/")
    model.learn(total_timesteps=50000000,callback=eval_callback)
    model.save("ppo_env.zip")

    model = PPO.load("logs/best_model/best_model.zip", env=test_env)


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
            vd_log.append(test_env.env.vp)







    animate_cbf_robot(x_log, y_log, theta_log, cbf_log, v_log, delta_log, vd_log)
    #animate_cbf_robot_path_preview(x_log,y_log,cbf_log,obs_log)