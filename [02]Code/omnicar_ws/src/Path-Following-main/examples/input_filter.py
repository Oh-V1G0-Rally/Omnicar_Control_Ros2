from platform import system

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import pandas as pd
import yaml

from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from dynamic_systems.dynamic_systems import AffineSystem
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
import copy
from dynamic_systems.systems import input_filter

L=1


with open("../config/config_RL.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())

x0_log = []
x1_log = []
u0_log = []
u1_log = []
# === Simulation ===
dt = conf["dt_control"]
k = 0
u = np.array([10, 10])
uf = np.array([0, 0])
for t in range(int(1 * 1 / dt)):
    if k==5:
        u = -u
        k=0
    uf = input_filter(u,uf,25,dt)
    x0_log.append(uf[0])
    x1_log.append(uf[1])
    u0_log.append(u[0])
    u1_log.append(u[1])
    k+=1

# Generate a time array based on your simulation steps and dt
time = [i * dt for i in range(len(x0_log))]

# Create a figure with 2 subplots (one for states, one for controls)
plt.figure(figsize=(10, 6))

# --- Plot States ---
plt.subplot(2, 1, 1)
plt.plot(time, x0_log, label='x0', linewidth=2)
plt.plot(time, x1_log, label='x1', linewidth=2, linestyle='--')
plt.ylabel('State Values')
plt.title('System Variables Over Time')
plt.legend()
plt.grid(True)

# --- Plot Controls ---
plt.subplot(2, 1, 2)
plt.plot(time, u0_log, label='u0', linewidth=2)
plt.plot(time, u1_log, label='u1', linewidth=2, linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Control Inputs')
plt.legend()
plt.grid(True)

# Adjust layout to prevent overlap and display the plot
plt.tight_layout()
plt.show()



