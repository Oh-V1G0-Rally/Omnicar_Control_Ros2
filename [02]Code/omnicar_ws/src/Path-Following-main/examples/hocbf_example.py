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

# === Initial setup ===

#system = DoubleIntegrator()
system = Unicycle2ndOrder()


with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())

path = Path(conf)
dt = conf["dt_control"]
#system = DiffDrive(conf)
H_ = canonical2D([1, 1], np.rad2deg(0.0))
H = np.zeros((5, 5))
H[0:2, 0:2] = H_
p = np.array([0,0,0,0,0])
cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=1,
                        limits=(-4, 4, -4, 4), spacing=0.05)
cbf = path.update_barrier_function(cbf,n=5)

cbf_log = []
controller = HOCBFQuadraticQP(system, cbf, beta1=10,beta2=100, dt=dt)

# === Data storage ===
x_log = []
y_log = []
system.step(np.array([0,0]), dt)
n=0
# === Simulation ===
for _ in range(int(30 * 1 / dt)):
    path.get_high_order_terms(2)
    cbf = path.update_barrier_function(cbf,5)
    controller.cbf = cbf
    u = controller.get_control()
    system.step(u,dt)
    path.update_with_vref(2)
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
    #print(cbf(s[0:2]))
    if cbf(s)<0:
        n+=1
        print(cbf(s),n,u)

# === Animation ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlim(min(x_log) - 1, max(x_log) + 1)
ax.set_ylim(min(y_log) - 1, max(y_log) + 1)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation")
ax.grid(True)
cbf_obj = ConcaveQuadraticBarrier(**cbf_log[500])
cbf_artists = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='blue', linestyle='-')
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

ds = 40*2
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

    new_lines = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='green', linestyle='-')
    cbf_lines.extend(new_lines)

    return [line, robot_dot] + cbf_lines


ani = animation.FuncAnimation(
    fig=fig,
    func=update,
    frames=len(cbf_log),
    init_func=init,
    interval=dt*10000,

)

plt.show()