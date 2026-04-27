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
from controllers.clf_controllers import *
import copy


L=1
ROM = Unicycle(np.array([-3,0.1,0]),L=L)
sys = Unicycle2ndOrder([-3,0.1,0,0,0],L=L)

dt = 1/1000
H = np.eye(3)
H[2,2] = 0
p = np.array([0, 0, 0])
alpha = 4
eps0=24
cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=0.5,
                        limits=(-4, 4, -4, 4), spacing=0.05,ISSF=True ,beta1=alpha,eps0=eps0,lambd=0)

ROM_CBF = CBFQuadraticQP(ROM,cbf)

clf = CLFTrackingQP(3,2,lambd=10)

# === Data storage ===
x_log = []
y_log = []
cbf_log = []
k0_log = []  # New: To store desired control
xi_log = []  # New: To store actual velocities
n = 0

# === Simulation ===

u_nom = np.array([1, 0])
uf = np.array([0, 0])
x_init = ROM.get_state()
ROM.set_control(u_nom)
K = 20



for k in range(int(5 * 1 / dt)):
    x = sys.get_state()
    ROM.set_state(x[0:3])

    if k % 100 == 0:
        u_nom = u_nom + 0.1 * np.array([1,0])


    ROM.set_control(uf)
    k0 = ROM_CBF.get_smooth_control()
    k0 = k0 + uf

    x = sys.get_state()
    q = x[0:3]
    xi = x[3:5]

    # --- Store the velocities ---
    k0_log.append(k0.copy())
    xi_log.append(xi.copy())
    # ----------------------------
    uf_dot = K * (u_nom - uf)
    q_dot = ROM.f(q) + ROM.g(q) @ xi

    grad_k0_q = ROM_CBF.get_sofptlus_gradient()
    grad_k0_uf = ROM_CBF.get_k0_gradient_uf(uf)
    grad_k0_gamma = ROM_CBF.get_k0_derivative_gamma(uf)
    term1 = grad_k0_q @ q_dot
    term2 = grad_k0_uf @ uf_dot
    term3 = (grad_k0_gamma*0)
    dt_k0 = term1 + term2 + term3
    dt_utotal = dt_k0 + uf_dot


    V = clf.get_V(q_dot, ROM.g(q) @ k0)
    u = clf.get_control(xi, k0, ROM.g(q), sys.g(x)[3:5, :], ROM.jacobian_g(q), grad_k0_q,grad_k0_uf,uf_dot,grad_k0_gamma,0,V)
    #print(u)

    h = ROM_CBF.cbf(q) - V
    print(V)

    sys.step(u, dt)
    uf = input_filter(u_nom, uf, K, dt)
    x_log.append(x[0])
    y_log.append(x[1])
    cbf_log.append({
        "hessian": copy.deepcopy(cbf.H),
        "center": copy.deepcopy(cbf.center),
        "height": copy.deepcopy(cbf.height),
        "limits": (cbf.center[0] - 2, cbf.center[0] + 2, cbf.center[1] - 2, cbf.center[1] + 2),
        "spacing": cbf.spacing
    })


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


# === Plotting k0 and xi ===
k0_arr = np.array(k0_log)
xi_arr = np.array(xi_log)

# Create a time array for the x-axis
t_arr = np.arange(len(k0_arr)) * dt

fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Subplot 1: Linear Velocity
ax1.plot(t_arr, k0_arr[:, 0], label='k0[0] (Desired v)', linestyle='--', color='r')
ax1.plot(t_arr, xi_arr[:, 0], label='xi[0] (Actual v)', alpha=0.7, color='b')
ax1.set_ylabel('Linear Velocity (m/s)')
ax1.set_title('CLF Tracking: Desired vs Actual Velocities')
ax1.legend()
ax1.grid(True)

# Subplot 2: Angular Velocity
ax2.plot(t_arr, k0_arr[:, 1], label='k0[1] (Desired w)', linestyle='--', color='r')
ax2.plot(t_arr, xi_arr[:, 1], label='xi[1] (Actual w)', alpha=0.7, color='b')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()
