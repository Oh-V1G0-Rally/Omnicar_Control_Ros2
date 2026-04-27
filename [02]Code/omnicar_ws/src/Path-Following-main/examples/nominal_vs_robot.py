import numpy as np
import yaml
from matplotlib import animation, pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback

from RL.Environment import *
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import dummy_vec_env, DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from gymnasium.wrappers import FlattenObservation

from plotting.plot_utils import  animate_cbf_robot

with open("../config/config.yaml", "rb") as f:
    conf = yaml.safe_load(f.read())





def make_env():
    return FlattenObservation(Environment(conf))
if __name__=="__main__":
    test_env = FlattenObservation(Environment(conf))



    model = PPO.load("logs/best_model/new_goat.zip", env=test_env)


    x_log, y_log, cbf_log, obs_log, theta_log = [], [], [], [], []
    v_log, delta_log, vd_log = [], [], []
    gamma_log, t_log = [], []
    path_gamma_log, t_path = [], []
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
            gamma_log.append(test_env.env.path.gamma)
            t_log.append(test_env.env.t)
    test_env.reset()
    t=0
    while True:
        test_env.env.path.update_with_vref(10)
        test_env.env.path.get_high_order_terms(10)
        path_gamma_log.append(test_env.env.path.gamma)
        t_path.append(t)
        t+=0.02
        print(t)
        if test_env.env.path.gamma>0.99:
            break






    animate_cbf_robot(x_log, y_log, theta_log, cbf_log, v_log, delta_log, vd_log)
    #animate_cbf_robot_path_preview(x_log,y_log,cbf_log,obs_log)

    import matplotlib.pyplot as plt

    # Plot agent gamma vs path gamma over time
    plt.figure(figsize=(10, 6))
    plt.plot(t_log, gamma_log, label="Agent gamma", linewidth=2)
    plt.plot(t_path, path_gamma_log, label="Path gamma", linewidth=2, linestyle="--")
    plt.xlabel("Time [s]")
    plt.ylabel("Gamma")
    plt.title("Agent Gamma vs Path Gamma")
    plt.legend()
    plt.grid(True)
    plt.show()