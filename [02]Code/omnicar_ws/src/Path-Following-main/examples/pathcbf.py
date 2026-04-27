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

L=1
system = Unicycle(np.array([0.5,0.0,np.deg2rad(45)]),L=L)

with open("../config/config_LCSS.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())
path = Path(conf)
dt = conf["dt_control"]
H = canonical2D([1, 1], np.rad2deg(0))
p = np.array([0, 0])


alpha = 1
radius = 0.9
desired_radius = 1
y = 0.5-0.5*desired_radius**2/radius**2
d = np.array([0.1, 0,0])
delta = np.linalg.norm(d)
lambd_issf= (1/(y-1))*np.log((-y*4*alpha)/(delta**2))
eps0 = np.exp(-lambd_issf)
cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=0.5,
                        limits=(-4, 4, -4, 4), spacing=0.05,ISSF=True ,beta1=alpha,eps0=eps0,delta=delta,lambd=lambd_issf)
cbf = path.update_barrier_function(cbf,n=3,radius=radius)
cbf_log = []

p_gamma = path.get_p_gamma_upper_bound(start=0.12,stop=0.2)
v_d= 0.008
u1_bounds=(-4,4)
u2_bounds=(-0.5,0.5)
ub = [u1_bounds[1],u2_bounds[1]]
lb = [u1_bounds[0],u2_bounds[0]]
delta_lb = path.get_delta_min(alpha=alpha,v_d_ub=v_d,u1_bounds=u1_bounds,u2_bounds=u2_bounds,lambd=cbf.H[0,0],eps0=eps0,lambd_issf=lambd_issf,L=L)
controller = PathCBFQuadraticQP(system, cbf,delta_lb=delta_lb,ub=ub,lb=lb)
path.reset()
path.gamma=0.12
s_0=path.get_spline_pose(path.gamma)
system.set_state(np.array(s_0))
# === Data storage ===
x_log = []
y_log = []
t_log = []
v_log = []
w_log = [ ]
theta_log = []
delta_log = []
correction_log = []
max_correction_log = []

tv_x = []
tv_y = []
n = 0

# === Simulation ===

for t in range(int(300 * 1 / dt)):
    s = system.get_state()

    path.get_high_order_terms(gamma_dot=v_d)
    cbf = path.update_barrier_function(cbf,3,radius=radius)
    controller.cbf = cbf
    u = controller.get_control()
    slack = controller.get_slack()
    nablah = cbf.gradient(s)
    d = [0,-0.2*u[1]]#-1*delta*nablah/(np.linalg.norm(nablah)) #[0,-u[1]*0.05]#
    if u is None:
        h_gamma = controller.cbf.time_derivative(s,gamma_dot=1)
        print("FAIL", slack+(v_d+delta_lb)*h_gamma, slack)
        print(cbf(s))
        break
    system.step(u[0:2]+d[0:2],dt)
    path.update_with_gamma(path.dgamma_dt+u[2])
    s = system.get_state()
    x_log.append(s[0])
    y_log.append(s[1])
    t_log.append(t*dt)
    v_log.append(u[0])
    w_log.append(u[1])
    theta_log.append(s[2])
    delta_log.append(u[2]*460)
    correction_log.append(u[2]*p_gamma)
    max_correction_log.append(delta_lb*p_gamma)


    cbf_log.append({
        "hessian": copy.deepcopy(cbf.H),
        "center": copy.deepcopy(cbf.center),
        "height": copy.deepcopy(cbf.height),
        "limits": (cbf.center[0] - 2, cbf.center[0] + 2, cbf.center[1] - 2, cbf.center[1] + 2),
        "spacing": cbf.spacing
    })
    #print(u)
    if cbf(s)<0:
        n+=1
        print(cbf(s),n,u)

    if path.gamma >0.20:
        print(t*dt)
        break

system = Unicycle(np.array([0.5,0.0,np.deg2rad(45)]),L=L)
path = Path(conf)

alpha = 1
radius = 0.9
desired_radius = 1
y = 0.5-0.5*desired_radius**2/radius**2
d = np.array([0.1, 0,0])
delta = np.linalg.norm(d)
lambd_issf= (1/(y-1))*np.log((-y*4*alpha)/(delta**2))
eps0 = np.exp(-lambd_issf)
cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=0.5,
                        limits=(-4, 4, -4, 4), spacing=0.05,ISSF=True ,beta1=alpha,eps0=eps0,delta=delta,lambd=lambd_issf)
cbf = path.update_barrier_function(cbf,n=3,radius=radius)
u1_bounds=(-4,4)
u2_bounds=(-0.5,0.5)
ub = [u1_bounds[1],u2_bounds[1]]
lb = [u1_bounds[0],u2_bounds[0]]
controller = CBFQuadraticQP(system, cbf)
path.reset()
path.gamma=0.12
s_0=path.get_spline_pose(path.gamma)
system.set_state(np.array(s_0))
# === Data storage ===
tv_x = []
tv_y = []
n = 0

# === Simulation ===

for t in range(int(300 * 1 / dt)):
    s = system.get_state()

    path.get_high_order_terms(gamma_dot=v_d)
    cbf = path.update_barrier_function(cbf,3,radius=radius)
    controller.cbf = cbf
    u = controller.get_control()
    #u = path.follow_path(s[0],s[1],s[2])
    u[0] = np.clip(u[0],u1_bounds[0],u1_bounds[1])
    u[1] = np.clip(u[1],u2_bounds[0],u2_bounds[1])
    nablah = cbf.gradient(s)
    d = [0,-0.2*u[1]]#-1*delta*nablah/(np.linalg.norm(nablah)) #[0,-u[1]*0.05]#
    if u is None:
        h_gamma = controller.cbf.time_derivative(s,gamma_dot=1)
        print(cbf(s))
        break
    system.step(u[0:2]+d[0:2],dt)
    path.update_with_gamma(path.dgamma_dt)
    s = system.get_state()
    tv_x.append(s[0])
    tv_y.append(s[1])



    cbf_log.append({
        "hessian": copy.deepcopy(cbf.H),
        "center": copy.deepcopy(cbf.center),
        "height": copy.deepcopy(cbf.height),
        "limits": (cbf.center[0] - 2, cbf.center[0] + 2, cbf.center[1] - 2, cbf.center[1] + 2),
        "spacing": cbf.spacing
    })
    #print(u)
    if cbf(s)<0:
        n+=1
        print(cbf(s),n,u)

    if path.gamma >0.20:
        print(t*dt)
        break


# === Load CSV ===


gammas = np.linspace(0, 1, 2000, endpoint=True)
path = interpolate.splev(gammas, path.tck_center)


# === Extract centerline coordinates ===
x = path[0]
y = path[1]
w_r = radius #df['w_tr_right_m'].values
w_l = radius #df['w_tr_left_m'].values

# === Compute tangents and normals ===
# Forward difference (approximate tangent)
dx = np.gradient(x)
dy = np.gradient(y)
norms = np.sqrt(dx**2 + dy**2)
dx /= norms
dy /= norms

# Normal vectors (perpendicular to tangent)
nx = -dy
ny = dx

# === Compute boundary lines ===
left_x = x + nx * w_l
left_y = y + ny * w_l
right_x = x - nx * w_r
right_y = y - ny * w_r

left_x2 = x + nx * desired_radius
left_y2 = y + ny * desired_radius
right_x2 = x - nx * desired_radius
right_y2 = y - ny * desired_radius

# === Animation ===
plt.rcParams.update({'font.size': 13})
fig, ax = plt.subplots(figsize=(9, 5.5))
ax.set_aspect('equal')
# Note: x_log, y_log, etc. need to be defined before this step
ax.set_xlim( -1, max(x_log) + 1-min(x_log))
ax.set_ylim( -1, max(y_log) + 1 - min(y_log))
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Vehicle Trajectories with the PCBF and TV-CBF Min-Norm Controllers")
ax.grid(True)

# Main Plot
ax.plot(x_log-min(x_log), y_log-min(y_log),label="PCBF Trajectory", linewidth=2)
ax.plot(tv_x-min(x_log), tv_y-min(y_log),label="TV-CBF Trajectory")
ax.plot(left_x-min(x_log), left_y-min(y_log), 'r--', label=r'$\partial \mathcal{C}$')
ax.plot(right_x-min(x_log), right_y-min(y_log), 'r--')
ax.plot(left_x2-min(x_log), left_y2-min(y_log), 'k--', label=r'$\partial \mathcal{C}_\delta$')
ax.plot(right_x2-min(x_log), right_y2-min(y_log), 'k--')

ax.legend(loc='upper left')

# ==========================================
# INSET AXIS SETUP
# ==========================================
# [x0, y0, width, height] in normalized axes coordinates (0 to 1)
# x0 = 0.35 (starts slightly left of center to center a 0.3 wide box)
# y0 = 0.05 (slightly above the bottom edge to avoid overlapping the main x-axis)
# width = 0.3 (30% of main plot width), height = 0.3 (30% of main plot height)
axins = ax.inset_axes([0.35, 0.05, 0.2, 0.4])

# 1. Plot the same data on the inset axis
axins.plot(x_log-min(x_log), y_log-min(y_log), linewidth=2)
axins.plot(left_x-min(x_log), left_y-min(y_log), 'r--', linewidth=2)
axins.plot(right_x-min(x_log), right_y-min(y_log), 'r--', linewidth=2)
axins.plot(left_x2-min(x_log), left_y2-min(y_log), 'k--', linewidth=2)
axins.plot(right_x2-min(x_log), right_y2-min(y_log), 'k--', linewidth=2)
axins.grid(True)
axins.set_xticklabels([])
axins.set_yticklabels([])

# 2. Set the specific zoom limits for the inset
# Change these values to the specific x and y coordinates you want to zoom in on
zoom_x_min, zoom_x_max = 14, 14.5
zoom_y_min, zoom_y_max = 7, 8
axins.set_xlim(zoom_x_min, zoom_x_max)
axins.set_ylim(zoom_y_min, zoom_y_max)

# 3. (Optional) Draw lines connecting the zoomed-in area on the main plot to the inset
ax.indicate_inset_zoom(axins, edgecolor="black")
# ==========================================

plt.show()


fig, ax = plt.subplots(figsize=(10, 5.5))
#ax.set_xlim(min(y_log) - 1, max(y_log) + 1)
#ax.set_ylim(0,max(t_log))
# Plot δ(t)
ax.plot(t_log, correction_log, label=r'$ u_\gamma(t) \| \bar p_{d\gamma} \| $', color='C0', linewidth=2)

# Lower bound
#ax.axhline(y=delta_lb*path.get_p_gamma_upper_bound(), color='C0', linestyle='--', linewidth=1.2, alpha=0.8, label=r'$u_{\gamma min}| p _\gamma\|$')
ax.plot(t_log, max_correction_log, color='C0', linestyle='--', linewidth=1.2, alpha=0.8, label=r'$ u_{\gamma min} \| \bar p_{d\gamma} \| $')

# Plot v(t) and ω(t)
ax.plot(t_log, v_log, label=r'$u_v(t)$', color='C3', linewidth=1.5)
ax.plot(t_log, w_log, label=r'$u_\omega(t)$', color='C1', linewidth=1.5)

# Labels and title
ax.set_xlabel(r'Time (s)', fontsize=12)
ax.set_ylabel(r'Signal value', fontsize=12)
ax.set_title(r'Path Speed Adjustment $u_\gamma(t)\|\bar p_{d \gamma}\|$ and Control Inputs $u_v(t)$, $u_\omega(t)$ Over Time')

# Grid and legend
ax.grid(True, which='both', linestyle='--', linewidth=0.7, alpha=0.7)
ax.legend(frameon=True, loc='best', fontsize=12)

plt.tight_layout()
plt.show()


















# # === Animation with CBF and normalized control bars ===
# fig, ax = plt.subplots(figsize=(8, 8))
# ax.set_aspect('equal')
# ax.set_xlim(min(x_log) - 3, max(x_log) + 3)
# ax.set_ylim(min(y_log) - 3, max(y_log) + 3)
# ax.set_xlabel("X (m)")
# ax.set_ylabel("Y (m)")
# ax.set_title("CBF Robot Animation")
# ax.grid(True)
#
# # Initialize artists
# line, = ax.plot([], [], 'b-', lw=2, label='Path')
# robot_dot, = ax.plot([], [], 'ro', markersize=2, label='Robot')
# ax.legend()
#
# # Store the CBF lines in a list
# cbf_lines = []
#
# ds = 10*2
# x_log = x_log[::ds]
# y_log = y_log[::ds]
# cbf_log = cbf_log[::ds]
#
# # Convert control logs to numpy arrays for easy operations
# v_log = np.array(v_log[::ds])
# w_log = np.array(w_log[::ds])
# delta_log = np.array(delta_log[::ds])
#
# # Compute maximum absolute value across all controls
# max_control = [np.max(np.abs(v_log)),
#     np.max(np.abs(w_log)),
#     np.max(np.abs(delta_log))]
#
#
# # Bar positions and size (bottom-right corner)
# x0 = ax.get_xlim()[1] - 0.05 * (ax.get_xlim()[1] - ax.get_xlim()[0]) - 2*0.08*(ax.get_xlim()[1] - ax.get_xlim()[0])
# y0 = ax.get_ylim()[0] + 0.10 * (ax.get_ylim()[1] - ax.get_ylim()[0])
# bar_spacing = 0.08 * (ax.get_xlim()[1] - ax.get_xlim()[0])
# bar_width = 0.05 * (ax.get_xlim()[1] - ax.get_xlim()[0])
# bars = []
# bar_labels = []
#
# for i in range(3):
#     bar = ax.bar(x0 + i * bar_spacing, 0, width=bar_width, color=f'C{i+2}', alpha=0.7, align='edge')
#     bars.append(bar)
#     # Add text label below each bar
#     label = ax.text(x0 + i * bar_spacing + bar_width/2, y0 - 0.05*(ax.get_ylim()[1]-ax.get_ylim()[0]),
#                     ['$u_v$', '$u_\\omega$', '$u_\gamma$'][i],
#                     ha='center', va='top', fontsize=10)
#     bar_labels.append(label)
#
#
# def init():
#     line.set_data([], [])
#     robot_dot.set_data([], [])
#     for bar in bars:
#         bar[0].set_height(0)
#         bar[0].set_y(y0)
#     return [line, robot_dot] + [b[0] for b in bars]
#
# def update(frame):
#     # Update path trace
#     line.set_data(x_log[:frame + 1], y_log[:frame + 1])
#     robot_dot.set_data([x_log[frame]], [y_log[frame]])
#
#     # Clear previous CBF lines
#     while cbf_lines:
#         cbf_lines.pop().remove()
#
#     # Plot new CBF lines
#     cbf_obj = ConcaveQuadraticBarrier(**cbf_log[frame])
#     new_lines = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='green', linestyle='-')
#     cbf_lines.extend(new_lines)
#
#
#     # Update bars (normalized to max_control)
#     controls = [v_log[frame], w_log[frame], delta_log[frame]]
#     for i, bar in enumerate(bars):
#         value = controls[i] / max_control[i]  # normalize to [-1, 1]
#         bar_height = value * 0.05 * (ax.get_ylim()[1] - ax.get_ylim()[0])  # scale to 20% of y-axis
#         bar[0].set_height(abs(bar_height))
#         bar[0].set_y(y0 if value >= 0 else y0 + bar_height)
#
#     return [line, robot_dot] + cbf_lines + [b[0] for b in bars]
#
# # Keep a reference to prevent garbage collection
# anim = animation.FuncAnimation(
#     fig=fig,
#     func=update,
#     frames=len(cbf_log),
#     init_func=init,
#     interval=dt*1000,
# )
#
# anim.save('cbf_robot_animation.mp4', writer='ffmpeg', fps=30, dpi=200)
# plt.show()


# === Animation with camera-following (agent-tracking) ===
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("CBF Robot Animation - Camera Follow")
ax.grid(True)

# Initialize artists
robot_dot, = ax.plot([], [], 'ro', markersize=4, label='Robot')
line, = ax.plot([], [], 'b-', lw=2, label='Path')  # optional, for local path segment
ax.legend()

cbf_lines = []
ds =10*2
x_log = x_log[::ds]
y_log = y_log[::ds]
theta_log = np.array(theta_log[::ds])
cbf_log = cbf_log[::ds]

v_log = np.array(v_log[::ds])
w_log = np.array(w_log[::ds])
delta_log = np.array(delta_log[::ds])
max_control = [np.max(np.abs(v_log)),
    np.max(np.abs(w_log)),
    np.max(np.abs(delta_log))]
# Bars at bottom-right
x0 = 0.8*(x_log[0])  # placeholder, will scale with axes
y0 = 0.05
bar_spacing = 0.08*(x_log[0])
bar_width = 0.05 * (20)
bars = []
for i in range(3):
    bar = ax.bar(0, 0, width=bar_width, color=f'C{i+2}', alpha=0.7, align='edge')
    bars.append(bar)
bar_labels = [ax.text(0,0,['$u_v$', '$u_\\omega$', '$u_\gamma$'][i], ha='center', va='top') for i in range(3)]

# Window size for camera (half-width/half-height)
window_x = 10
window_y = 10

# Initialize the arrow
arrow = None

def init():
    robot_dot.set_data([], [])
    line.set_data([], [])
    for bar in bars:
        bar[0].set_height(0)
        bar[0].set_y(0)
    return [robot_dot, line] + [b[0] for b in bars] + bar_labels

def update(frame):
    global arrow  # must be first line

    # Robot position
    x, y = x_log[frame], y_log[frame]
    robot_dot.set_data([x], [y])

    # Local path (last 20 points)
    start_idx = max(0, frame-20)
    line.set_data(x_log[start_idx:frame+1], y_log[start_idx:frame+1])

    # Camera follows robot
    ax.set_xlim(x - window_x, x + window_x)
    ax.set_ylim(y - window_y, y + window_y)

    # Clear previous CBF lines
    while cbf_lines:
        cbf_lines.pop().remove()
    plt.plot(left_x, left_y, 'r--', label='Track Limits')
    plt.plot(right_x, right_y, 'r--')
    cbf_obj = ConcaveQuadraticBarrier(**cbf_log[frame])
    new_lines = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='green', linestyle='-')
    cbf_lines.extend(new_lines)

    # Update arrow for orientation
    if arrow is not None:
        arrow.remove()
    theta = theta_log[frame]
    dx = np.cos(theta)*0.5
    dy = np.sin(theta)*0.5
    arrow = ax.arrow(x, y, dx, dy, head_width=0.3, head_length=0.4, fc='k', ec='k')

    # Update control bars (bottom-right)
    ax_xlim = ax.get_xlim()
    ax_ylim = ax.get_ylim()
    x0 = ax_xlim[0] + 0.75*(ax_xlim[1]-ax_xlim[0])
    y0 = ax_ylim[0] + 0.15*(ax_ylim[1]-ax_ylim[0])
    controls = [v_log[frame], w_log[frame], delta_log[frame]]
    for i, bar in enumerate(bars):
        value = controls[i] / max_control[i]
        bar_height = value * 0.15 * (ax_ylim[1]-ax_ylim[0])
        bar[0].set_height(abs(bar_height))
        bar[0].set_y(y0 if value>=0 else y0+bar_height)
        bar[0].set_x(x0 + i*0.08*(ax_xlim[1]-ax_xlim[0]))
        bar_labels[i].set_position((x0 + i*0.08*(ax_xlim[1]-ax_xlim[0]) + 0.025*(ax_xlim[1]-ax_xlim[0]),
                                    y0 - 0.02*(ax_ylim[1]-ax_ylim[0])))

    return [robot_dot, line, arrow] + cbf_lines + [b[0] for b in bars] + bar_labels

anim = animation.FuncAnimation(
    fig=fig,
    func=update,
    frames=len(cbf_log),
    init_func=init,
    interval=dt*1000,
)
#anim.save('cbf_third_person_animation.mp4', writer='ffmpeg', fps=30, dpi=200)
#plt.show()

