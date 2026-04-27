import matplotlib.pyplot as plt
from controllers.clf_controllers import NewCLFCBFQuadraticQP
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
import copy
import numpy as np


system = Integrator(n=2, state=np.array([-0.1 , -1]))
with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())

dt = conf["dt_control"]
H = np.diag([1,1])
xd = np.array([0, 0])
clf = QuadraticLyapunov(hessian=H, center=xd, height=0,
                        limits=(-4, 4, -4, 4), spacing=0.05, alpha=0.5)
H = canonical2D([5, 20], np.rad2deg(0))
p = np.array([0, -0.5])
cbf = QuadraticBarrier(hessian=H, center=p, height=1,
                        limits=(-4, 4, -4, 4), spacing=0.05,beta1=1)


cbf_log = []
controller = NewCLFCBFQuadraticQP(system, clf,[cbf])
s = system.get_state()

# === Data storage ===
x_log = []
y_log = []
active_clf_log = []
n = 0
#=== Simulation ===

for k in range(int(20* 1 / dt)):

    u = controller.get_control()
    print(u,s,controller.active_clfs, controller.cbfs[0](s))
    #print(u)
    system.step(u,dt)
    s = system.get_state()
    #print(s,u,controller.cbfs[0](s))
    x_log.append(s[0])
    y_log.append(s[1])
    cbf_log.append({
        "hessian": copy.deepcopy(cbf.H),
        "center": copy.deepcopy(cbf.center),
        "height": copy.deepcopy(cbf.height),
        "limits": (cbf.center[0] - 2, cbf.center[0] + 2, cbf.center[1] - 2, cbf.center[1] + 2),
        "spacing": cbf.spacing
    })
    active_clf_log.append(controller.active_clfs)
    if np.linalg.norm(s-xd) <1e-1:
        break



# === Animation ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlim(min(x_log) - 1, max(x_log) + 1)
ax.set_ylim(min(y_log) - 1, max(y_log) + 1)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation")
ax.grid(True)

path = np.array(controller.path)
plt.plot(x_log, y_log)
ax.scatter(path[:, 0], path[:, 1], color='green', s=15, alpha=0.5, label="Possible CLFs")
ax.scatter(0, 0, color='green', s=15, alpha=0.5,label="goal")
cbf_obj = QuadraticBarrier(**cbf_log[0])
cbf_artists = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='blue', linestyle='-')
plt.show()

from matplotlib.animation import FuncAnimation

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# === Setup Figure ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')

# Dynamic limits
ax.set_xlim(min(x_log) - 1, max(x_log) + 1)
ax.set_ylim(min(y_log) - 1, max(y_log) + 1)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation")
ax.grid(True)

# === 1. Plot Static Elements ===
ax.scatter(path[:, 0], path[:, 1], color='green', s=15, alpha=0.5, label="Possible CLFs")
ax.scatter(0, 0, color='green', s=15, alpha=1,label="Goal")

cbf_obj = QuadraticBarrier(**cbf_log[0])
cbf_obj.plot_levels(ax=ax, levels=[0.0], color='blue', linestyle='-', label="Barrier")

# === 2. Initialize Dynamic Elements ===
# Robot (Red dot)
robot_marker, = ax.plot([], [], 'ro', markersize=10, zorder=5, label="Robot")

# Trail (Black line)
trail_line, = ax.plot([], [], 'k-', linewidth=1, alpha=0.6)

# Active CLF Target (Gold Star) - NEW
clf_target, = ax.plot([], [], 'y*', markersize=12, zorder=6, label="Active CLF")

ax.legend(loc='upper right')


# === 3. Animation Functions ===
def init():
    robot_marker.set_data([], [])
    trail_line.set_data([], [])
    clf_target.set_data([], [])
    return robot_marker, trail_line, clf_target


def update(frame):
    # 1. Update Robot & Trail
    x_curr = x_log[frame]
    y_curr = y_log[frame]

    robot_marker.set_data([x_curr], [y_curr])
    trail_line.set_data(x_log[:frame + 1], y_log[:frame + 1])

    # 2. Update Active CLF Logic - NEW
    current_activation = active_clf_log[frame]

    # Find all indices where value is non-zero/True
    active_indices = np.flatnonzero(current_activation)

    if len(active_indices) == 0:
        # Case: All zeros -> scatter at (0,0)
        clf_target.set_data([0], [0])
    else:
        # Case: Active -> get highest index
        highest_idx = active_indices[-1]

        # User Logic: scatter at path[i-1]
        # Ensure we don't go out of bounds if i=0
        path_idx = max(0, highest_idx)

        target_x = path[path_idx, 0]
        target_y = path[path_idx, 1]

        clf_target.set_data([target_x], [target_y])

    return robot_marker, trail_line, clf_target


# === 4. Run Animation ===
ani = FuncAnimation(
    fig,
    update,
    frames=len(x_log),
    init_func=init,
    interval=2,
    blit=True
)
#print("Saving animation... this may take a moment.")
#ani.save("chattering.mp4", writer="ffmpeg", fps=120, dpi=200)
#print("Saved as robot_cbf_animation.mp4")
plt.show()