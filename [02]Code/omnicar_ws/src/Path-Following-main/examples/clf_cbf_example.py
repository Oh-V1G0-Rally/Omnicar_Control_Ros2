from platform import system

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import yaml

from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from controllers.clf_controllers import CLFQuadraticQP, CLFCBFQuadraticQP
from dynamic_systems.dynamic_systems import AffineSystem
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
import copy
import numpy as np


system = Integrator(n=2, state=np.array([0.4 , -0.6566667]))
with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())

dt = conf["dt_control"]
H = np.diag([1,1]) #canonical2D([10, 10,10], np.rad2deg(0))
xd = np.array([0, 0])
clf = QuadraticLyapunov(hessian=H, center=xd, height=0,
                        limits=(-4, 4, -4, 4), spacing=0.05, alpha=0.5)

H = canonical2D([5, 20], np.rad2deg(0))
p = np.array([0, -0.5])
cbf = QuadraticBarrier(hessian=H, center=p, height=1,
                        limits=(-4, 4, -4, 4), spacing=0.05,beta1=1)


cbf_log = []
controller = CLFCBFQuadraticQP(system, [clf],[cbf])

# === Data storage ===
x_log = []
y_log = []
n = 0
# === Simulation ===

for k in range(int(20* 1 / dt)):

    u = controller.get_control()
    system.step(u,dt)
    s = system.get_state()
    print(s,u)
    x_log.append(s[0])
    y_log.append(s[1])
    cbf_log.append({
        "hessian": copy.deepcopy(cbf.H),
        "center": copy.deepcopy(cbf.center),
        "height": copy.deepcopy(cbf.height),
        "limits": (cbf.center[0] - 2, cbf.center[0] + 2, cbf.center[1] - 2, cbf.center[1] + 2),
        "spacing": cbf.spacing
    })

    #if 0 in controller.get_active_constraints():
     #  break



# === Animation ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlim(min(x_log) - 2, max(x_log) + 2)
ax.set_ylim(min(y_log) - 2, max(y_log) + 2)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation")
ax.grid(True)

plt.plot(x_log, y_log)
cbf_obj = QuadraticBarrier(**cbf_log[0])
cbf_artists = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='blue', linestyle='-')
plt.show()





import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


# === Setup Figure ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')

# Dynamic limits
ax.set_xlim(min(x_log) - 0.3, max(x_log) + 0.3)
ax.set_ylim(min(y_log) - 0.3, max(y_log) + 0.3)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation")
ax.grid(True)

# === 1. Plot Static Elements ===
ax.scatter(0, 0, color='green', s=15, alpha=1,label="Goal")

cbf_obj = QuadraticBarrier(**cbf_log[0])
cbf_obj.plot_levels(ax=ax, levels=[0.0], color='blue', linestyle='-', label="Barrier")

# === 2. Initialize Dynamic Elements ===
# Robot (Red dot)
robot_marker, = ax.plot([], [], 'ro', markersize=10, zorder=5, label="Robot")

# Trail (Black line)
trail_line, = ax.plot([], [], 'k-', linewidth=1, alpha=0.6)


ax.legend(loc='upper right')


# === 3. Animation Functions ===
def init():
    robot_marker.set_data([], [])
    trail_line.set_data([], [])
    return robot_marker, trail_line


def update(frame):
    # 1. Update Robot & Trail
    x_curr = x_log[frame]
    y_curr = y_log[frame]

    robot_marker.set_data([x_curr], [y_curr])
    trail_line.set_data(x_log[:frame + 1], y_log[:frame + 1])

    return robot_marker, trail_line


# === 4. Run Animation ===
ani = FuncAnimation(
    fig,
    update,
    frames=len(x_log),
    init_func=init,
    interval=7,
    blit=True
)
#print("Saving animation... this may take a moment.")
#ani.save("robot_cbf_animation.mp4", writer="ffmpeg", fps=120, dpi=200)
#print("Saved as robot_cbf_animation.mp4")
plt.show()