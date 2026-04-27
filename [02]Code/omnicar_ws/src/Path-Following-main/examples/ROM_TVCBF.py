import time
from platform import system
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import yaml
import copy

from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from controllers.clf_controllers import CLFTrackingQP
from dynamic_systems.dynamic_systems import AffineSystem
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *


def build_qp_matrices(A, d_bounds, eta, uf_dot, vd_dot, grad_k0_uf, grad_k0_vd, dt):
    # Dimensions
    dim_u = 2  # Dimension of uf (e.g., [v, w])
    dim_v = 1  # Dimension of vd
    dim_x = dim_u + dim_v

    # 1. Objective Matrices (min ||x||^2)
    P =  np.array([[0.1, 0, 0], [0.0 , 1.0 , 0.0], [0, 0 , 0.001]])
    Q = np.zeros(dim_x)

    # Identity matrix for the (I + grad_k0_uf) term
    I_u = np.eye(dim_u)

    # Pre-compute the shared gradient block: (I + grad_k0_uf)
    grad_u_block = I_u + grad_k0_uf

    # Ensure grad_k0_vd is a column vector, shape (2, 1)
    grad_k0_vd_col = grad_k0_vd.reshape(-1, 1)

    # 2. Build C (The constraint matrix)
    # Horizontally stack the multipliers and scale the WHOLE block by dt
    inner_C = np.hstack((grad_u_block, grad_k0_vd_col)) * dt
    C = A @ inner_C

    # 3. Build b (The bounds vector)
    # Calculate the known drift/nominal terms
    # grad_u_block @ uf_dot yields shape (2,)
    # (grad_k0_vd_col * vd_dot).flatten() yields shape (2,)
    derivative_terms = (grad_u_block @ uf_dot) + (grad_k0_vd_col * vd_dot).flatten()

    known_terms = eta + (derivative_terms * dt)

    b = d_bounds - A @ known_terms

    return P, Q, C, b


def map_range(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# ==========================================
# SETUP & INITIALIZATION
# ==========================================
L = 0.5
ROM = Unicycle(np.array([0.5, 0.0, np.deg2rad(45)]), L=L)
sys = Unicycle2ndOrder([0.5, 0.0, np.deg2rad(45), 0, 0], L=L)

with open("../config/config_LCSS.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())
path = Path(conf)
dt = conf["dt_control"]

H = np.eye(3)
H[2, 2] = 0
p = np.array([0, 0, 0])
alpha = 1
radius = 1
K = 10
eps0 = 24
lambda_issf = -np.log(eps0)
max_v = 10.0
max_w = 2.0
max_vp = 10.0
u1_bounds = np.array([-max_v, max_v])
u2_bounds = np.array([-max_w, max_w])
ub = np.array([u1_bounds[1], u2_bounds[1]])
lb = np.array([u1_bounds[0], u2_bounds[0]])
A = np.array([
    [1.0, 0.0],  # u1 <= max_v
    [-1.0, 0.0],  # -u1 <= max_v
    [0.0, 1.0],  # u2 <= max_w
    [0.0, -1.0]  # -u2 <= max_w
])

d_bounds = np.array([
    max_v,
    0,
    max_w,
    max_w
])


cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=0.5,
                              limits=(-4, 4, -4, 4), spacing=0.05, is_ISSF=True, beta1=alpha, eps0=eps0, lambd=lambda_issf,ub=ub,lb=lb)
cbf = path.update_barrier_function(cbf, n=3, radius=radius)
clf = CLFTrackingQP(3, 2, lambd=10)
v_d = path.vref_to_gamma_dot(max_vp)

p_gamma = path.get_p_gamma_upper_bound(start=10,stop=2000)
delta_lb = path.get_delta_min(alpha=alpha,v_d_ub=v_d,u1_bounds=u1_bounds,u2_bounds=u2_bounds,lambd=cbf.H[0,0],eps0=eps0,lambd_issf=lambda_issf,L=L)


ROM_CBF = CBFQuadraticQP(ROM, cbf)
path.reset()
path.gamma = 0
s_0 = path.get_spline_pose(path.gamma)
sys.set_state([s_0[0] + 0.1, s_0[1], s_0[2], 0, 0])
ROM.set_state(sys.get_state()[0:3])
path.get_high_order_terms(gamma_dot=0)
cbf = path.update_barrier_function(cbf, 3, radius=radius)
ROM_CBF.cbf = cbf

# === Data storage ===
x_log = []
y_log = []
cbf_log = []
k0_log = []
xi_log = []

# === RL Environment Variables (Replaces self.*) ===

input_filter_K = K
vd_f = 0
t = 0.0

u_nom = np.array([0.0, 0.0])
uf = np.array([0.0, 0.0])

print("Starting simulation...")

# ==========================================
# SIMULATION LOOP
# ==========================================
num_rl_steps = 2000*4

for rl_step in range(num_rl_steps):

    t_current = rl_step * (10 * dt)
    frequency = 1  # 10 seconds per full wave cycle
    action_2_sine = np.sin(2 * np.pi * frequency * t_current)
    action_1_cos = np.cos(2 * np.pi * frequency * t_current)

    # [v_command, w_command, path_velocity_command]
    action = [action_1_cos, 0, action_2_sine]

    truncated = False
    fail = False

    v = map_range(action[0], -1, 1, -max_v, max_v)
    delta = map_range(action[1], -1, 1, -max_w, max_w)
    u_nom = np.array([v, delta])
    vp_temp = map_range(action[2], -1, 1, 0, max_vp)
    vd_nom = path.vref_to_gamma_dot(vp_temp)

    for i in range(4):
        x = sys.get_state()
        q = x[0:3]
        xi = x[3:5]
        path.get_high_order_terms(gamma_dot=vd_f)
        ROM_CBF.cbf = path.update_barrier_function(cbf, 3, radius=radius)
        ROM.set_control(uf)
        u_corr = ROM_CBF.get_smooth_control()
        k0 = u_corr
        u_total = k0 + uf

        uf_dot = input_filter_K * (u_nom - uf)
        vd_dot = (vd_nom - vd_f) * input_filter_K * 2
        grad_k0_q = ROM_CBF.get_sofptlus_gradient()
        grad_k0_uf = ROM_CBF.get_k0_gradient_uf(uf)
        grad_k0_gamma = ROM_CBF.get_k0_derivative_gamma(uf)
        grad_k0_vd = ROM_CBF.get_k0_derivative_vd(uf)
        x_dot = ROM.f(q) + ROM.g(q) @ xi

        # FORCE FLATTEN to block NumPy broadcasting traps
        dk0_dq = (grad_k0_q @ x_dot).flatten()
        dk0_dgamma = (grad_k0_gamma * vd_f).flatten()

        eta = u_total+(dk0_dq + dk0_dgamma)*dt


        P, Q, C, b =build_qp_matrices(A, d_bounds, eta, uf_dot, vd_dot, grad_k0_uf, grad_k0_vd,dt)
        QP = QuadraticProgram(P=P, q=Q)
        QP.set_inequality_constraints(C, b)
        sol = np.array(QP.get_solution())
        u_nom_correction = (sol[0:2] + uf_dot)/input_filter_K + uf - u_nom
        vd_nom_correction = (sol[2]+ vd_dot  ) /(input_filter_K*2) + vd_f - vd_nom
        u_nom_corrected = u_nom + u_nom_correction
        vd_corrected = vd_nom + vd_nom_correction

        uf_dot = input_filter_K * (u_nom_corrected - uf)
        vd_dot = (vd_corrected - vd_f) * input_filter_K * 2
        dk0_duf = (grad_k0_uf @ uf_dot).flatten()
        dk0_dvddot = (grad_k0_vd * vd_dot).flatten()

        dt_k0 = dk0_dq + dk0_duf + dk0_dgamma + dk0_dvddot
        dt_utotal = dt_k0 + uf_dot.flatten()


        q_dot = ROM.g(q)@xi
        V = clf.get_V(q_dot, ROM.g(q) @ u_total)
        print(V)

        u = clf.get_control(q_dot, xi, u_total, dt_utotal, ROM.g(q), ROM.jacobian_g(q), sys.g(x)[3:5, :], V)
        # print(ROM.get_state())

        h = ROM_CBF.cbf(q) - 1 / 15 * (V - 0.08 / alpha * 15)
        #print(ROM_CBF.cbf(q))
        # if h<0 or ROM_CBF.cbf(q) <-0.02059999999999995:
        #   print(h,ROM_CBF.cbf(q), t)

        # 6. Step all systems forward to time t + dt
        uf = input_filter(u_nom_corrected, uf, input_filter_K, dt)  # Steps uf forward
        sys.step(u, dt)  # Steps x forward
        path.update_with_gamma(vd_f)
        vd_f += (vd_corrected - vd_f) * input_filter_K * dt * 2
        path.get_high_order_terms(gamma_dot=vd_f)
        ROM_CBF.cbf = path.update_barrier_function(ROM_CBF.cbf, 3, radius=radius)
        ROM.set_state(sys.get_state()[0:3])

        t += dt
        vp = path.gamma_dot_to_vref(vd_f)

        # Logging Data
        x_next = ROM.get_state()
        x_log.append(x_next[0])
        y_log.append(x_next[1])
        cbf_log.append({
            "hessian": copy.deepcopy(cbf.H),
            "center": copy.deepcopy(cbf.center),
            "height": copy.deepcopy(cbf.height),
            "limits": (cbf.center[0] - 2, cbf.center[0] + 2, cbf.center[1] - 2, cbf.center[1] + 2),
            "spacing": cbf.spacing
        })
        k0_log.append(u_total.copy())
        xi_log.append(xi.copy())

# ==========================================
# PLOTTING
# ==========================================
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