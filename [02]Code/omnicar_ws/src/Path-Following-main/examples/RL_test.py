import os
import numpy as np
import yaml
import pandas as pd
from matplotlib import pyplot as plt
from stable_baselines3 import PPO
from gymnasium.wrappers import FlattenObservation

from RL.Environment import Environment
from plotting.plot_utils import animate_cbf_robot, plot_trajectories, \
    plot_controls  # make sure it accepts centerline argument

# === Base config ===
with open("../config/config.yaml", "rb") as f:
    base_conf = yaml.safe_load(f.read())

# === Root directory containing all track folders ===
tracks_root = "../splines"

# === Find all track YAML and CSV files ===
track_data = []
for root, dirs, files in os.walk(tracks_root):
    yaml_files = [f for f in files if f.endswith("_centerline_tck.yaml")]
    csv_files  = [f for f in files if f.endswith(".csv") and "centerline" in f.lower()]

    for yaml_file in yaml_files:
        # Try to find matching CSV
        csv_file = next((c for c in csv_files if os.path.splitext(c)[0].lower() in yaml_file.lower()), None)
        if csv_file:
            track_data.append({
                "name": os.path.basename(root),
                "yaml": os.path.join(root, yaml_file),
                "csv": os.path.join(root, csv_file)
            })

if not track_data:
    raise FileNotFoundError(f"No tracks with YAML + CSV found in {tracks_root}")

print(f"🏁 Found {len(track_data)} tracks:")

for t in track_data:
    print("  •", t["name"])

# === Environment factory ===
def make_env(conf):
    return FlattenObservation(Environment(conf))

# === Load RL model ===
model = PPO.load("logs/best_model/new_goat.zip")

# === Multi-track evaluation ===
i = 0
for track in track_data:
    print(f"\n Evaluating track: {track['name']}")

    # Update config
    conf = base_conf.copy()
    conf["map_path"] = tracks_root+"/"
    conf["track"] = track["name"]
    test_env = make_env(conf)

    # Logs
    x_log, y_log, cbf_log, obs_log, theta_log = [], [], [], [], []
    v_log, delta_log, vd_log = [], [], []
    v_corr_log, w_corr_log = [], []
    t_log = [ ]

    obs, info = test_env.reset()
    terminated, truncated = False, False

    while not (terminated or truncated):
        action, _ = model.predict(obs,deterministic=True)
        obs, reward, terminated, truncated, info = test_env.step(action)

        # Logging
        x_log.append(info["state"][0])
        y_log.append(info["state"][1])
        cbf_log.append(info["cbf"])
        obs_log.append(test_env.env.observation["path"])
        theta_log.append(info["state"][2])
        v_log.append(info["state"][3])
        delta_log.append(info["state"][4])
        vd_log.append(test_env.env.vd)
        t_log.append(test_env.env.t)
        v_corr_log.append(test_env.env.u_corr[0])
        w_corr_log.append(test_env.env.u_corr[1])


    # Animate robot + centerline
    print(f" Generating animation for {track['name']}...",i)
    print(test_env.env.path.gamma,test_env.env.t)

    #animate_cbf_robot(x_log, y_log, theta_log, cbf_log, v_log, delta_log, vd_log,centerline=track["csv"],obs_log=obs_log) # overlay centerline)
    #plot_trajectories(x_log,y_log,v_log,track["csv"])
    plot_controls(v_log,delta_log,vd_log,t_log)
    plt.close("all")
    i+=1
print("\n Evaluation complete for all tracks! ")