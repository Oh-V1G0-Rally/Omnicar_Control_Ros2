from platform import system

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import yaml

from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from dynamic_systems.dynamic_systems import AffineSystem
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
import copy


system = Integrator(n=2,state=np.array([-5,0.0]))
#system = Unicycle()
with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())

dt = 0.005
# Define Barrier Function
H = canonical2D([1, 1], np.rad2deg(0.0))
p = np.array([-2, 0.1])
alpha = 10
eps0 = 0.5
d = np.array([2, 0])
delta = np.linalg.norm(d)
lambd=0.1
cbf = QuadraticBarrier(hessian=H, center=p, height=1,
                       limits=(-1, 1, -1, 1), spacing=0.05,ISSF=True ,beta1=alpha,eps0=eps0,delta=delta,lambd=lambd)
gamma = 1/(alpha*4)*eps0*delta**2
print(gamma)
cbf_log = []
controller = CBFQuadraticQP(system, cbf)

# === Data storage ===
x_log = []
y_log = []
n = 0

for k in range(int(20* 1 / dt)):
    s = system.get_state()
    nablah = cbf.gradient(s)
    d = -1*delta* nablah/(np.linalg.norm(nablah))
    u_nom = -1*s
    system.set_control(u_nom)
    u = controller.get_control()
    system.step(u_nom + u +d, dt)
    s = system.get_state()
    x_log.append(s[0])
    y_log.append(s[1])
    cbf_log.append({
        "hessian": copy.deepcopy(cbf.H),
        "center": copy.deepcopy(cbf.center),
        "height": copy.deepcopy(cbf.height),
        "limits": (cbf.center[0] - 2, cbf.center[0] + 2, cbf.center[1] - 2, cbf.center[1] + 2),
        "spacing": cbf.spacing
    })
    if cbf(s) <0:
        print(cbf(s) + gamma)



# === Animation ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlim(min(x_log) - 1, max(x_log) + 1)
ax.set_ylim(min(y_log) - 1, max(y_log) + 1)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation")
ax.grid(True)
cbf_obj = ConcaveQuadraticBarrier(**cbf_log[0])
cbf_artists = cbf_obj.plot_levels(ax=ax, levels=[gamma], color='blue', linestyle='-')
cbf_artists1 = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='red', linestyle='-')
plt.plot(x_log,y_log)
plt.show()


# === Animation with CBF ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlim(min(x_log) - 1, max(x_log) + 1)
ax.set_ylim(min(y_log) - 1, max(y_log) + 1)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation")
ax.grid(True)

# Initialize artists
line, = ax.plot([], [], 'b-', lw=2, label='Path')
robot_dot, = ax.plot([], [], 'ro', markersize=1, label='Robot')
ax.legend()

# Store the CBF lines in a list
cbf_lines = []

ds = 2
x_log = x_log[::ds]
y_log = y_log[::ds]
cbf_log = cbf_log[::ds]
def init():
    line.set_data([], [])
    robot_dot.set_data([], [])
    return [line, robot_dot]


def update(frame):
    # Update path trace

    line.set_data(x_log[:frame + 1], y_log[:frame + 1])

    # Update robot position
    robot_dot.set_data([x_log[frame]], [y_log[frame]])
    # Clear previous CBF lines
    while cbf_lines:
        cbf_lines.pop().remove()

    # Plot new CBF lines
    cbf_obj = ConcaveQuadraticBarrier(**cbf_log[frame])

    new_lines = cbf_obj.plot_levels(ax=ax, levels=[gamma], color='green', linestyle='-')
    cbf_lines.extend(new_lines)

    return [line, robot_dot] + cbf_lines


ani = animation.FuncAnimation(
    fig=fig,
    func=update,
    frames=len(cbf_log),
    init_func=init,
    interval=dt,

)

plt.show()