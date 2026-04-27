from platform import system

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import yaml

from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from controllers.clf_controllers import CLFQuadraticQP
from dynamic_systems.dynamic_systems import AffineSystem
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
import copy
import numpy as np


system = Integrator(n=2, state=np.array([1, 0.5]))
with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())

dt = conf["dt_control"]

H = np.diag([1,1])#canonical2D([10, 10,10], np.rad2deg(0))
p = np.array([0, 0])
clf = QuadraticLyapunov(hessian=H, center=p, height=0,
                        limits=(-4, 4, -4, 4), spacing=0.05,alpha=4)


cbf_log = []
controller = CLFQuadraticQP(system, clf)

# === Data storage ===
x_log = []
y_log = []
n = 0
# === Simulation ===

for k in range(int(2* 1 / dt)):
    s = system.get_state()
    u_nom = (2* (- s))
    system.set_control(u_nom)
    u = controller.get_control()
    system.step(u_nom+u,dt)
    s = system.get_state()
    #print(u,u_nom)
    x_log.append(s[0])
    y_log.append(s[1])


# === Animation ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlim(min(x_log) - 1, max(x_log) + 1)
ax.set_ylim(min(y_log) - 1, max(y_log) + 1)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation")
ax.grid(True)

plt.plot(x_log, y_log)
plt.show()