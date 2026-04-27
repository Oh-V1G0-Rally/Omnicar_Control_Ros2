from platform import system
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import pandas as pd
import yaml
import copy

from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from dynamic_systems.dynamic_systems import AffineSystem
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *

# ==========================================
# SETUP & INITIALIZATION
# ==========================================
L = 0.5

ROM = Unicycle(np.array([0.5, 0.0, np.deg2rad(45)]), L=L)
sys = Unicycle2ndOrderMRAC([0.5, 0.0, np.deg2rad(45), 0, 0], L=L)

with open("../config/config_LCSS.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())
path = Path(conf)
dt = conf["dt_control"]
H = np.eye(3)
H[2, 2] = 0
p = np.array([0, 0, 0])
alpha = 1
radius = 1
eps0 = 0.1
K = 1

cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=0.5,
                              limits=(-4, 4, -4, 4), spacing=0.05, beta1=alpha, ISSF=True, eps0=eps0, lambd=0)
cbf = path.update_barrier_function(cbf, n=3, radius=radius)

# Base Path Velocity and Initial Filtered State
vd_base = 20.0
vd_f = vd_base  # NEW: The continuously filtered path velocity state

ROM_CBF = CBFQuadraticQP(ROM, cbf)
path.reset(gamma=1)

s_0 = path.get_spline_pose(path.gamma)
sys.set_state([s_0[0] + 0.1, s_0[1], s_0[2], 0, 0])
ROM.set_state(sys.get_state()[0:3])
path.get_high_order_terms(gamma_dot=vd_f)
cbf = path.update_barrier_function(cbf, 3, radius=radius)
ROM_CBF.cbf = cbf

# === Data storage ===
x_log = []
y_log = []
cbf_log = []

# === Initialization ===
u_nom = np.array([0.0, 0.0])
uf = np.array([0.0, 0.0])

x = ROM.get_state()
ROM.set_control(uf)
k0 = ROM_CBF.get_smooth_control()
u_total = k0 + uf

pred_k0 = np.copy(k0)
pred_utotal = np.copy(u_total)
pred_uf = np.copy(uf)

print("Starting simulation...")

# ==========================================
# MAIN SIMULATION LOOP
# ==========================================
for k in range(int(5 * 1 / dt)):
    t = k * dt

    # 1. Update Exogenous Input for current time t
    # Sine wave acting as the NOMINAL desired path velocity
    amplitude = 20
    frequency = 1  # Hz
    vd_nom = vd_base + amplitude * np.sin(2 * np.pi * frequency * t)

    # Calculate Acceleration (vd_dot) exactly using the First-Order Filter
    vd_dot = K * (vd_nom - vd_f)

    # NEW: Sine wave acting as the NOMINAL desired robot linear velocity
    u_amplitude = 10
    u_frequency = 1
    u_nom = np.array([u_amplitude * np.sin(2 * np.pi * u_frequency * t), 0.0])

    # 2. Get Current States and CBF Control
    x = ROM.get_state()
    ROM.set_control(uf)
    k0 = ROM_CBF.get_smooth_control()
    u_total = k0 + uf

    # 3. Validate Predictions
    error_k0 = np.linalg.norm(k0 - pred_k0)
    error_utotal = np.linalg.norm(u_total - pred_utotal)
    error_uf = np.linalg.norm(uf - pred_uf)

    if k % 10 == 0:
        print(f"Step {k:4d} | k0 Error: {error_k0:.2e} | u_total Error: {error_utotal:.2e} | uf Error: {error_uf:.2e}")

    # 4. Compute all Derivatives
    uf_dot = K * (u_nom - uf)

    grad_k0_x = ROM_CBF.get_sofptlus_gradient()
    f_x = ROM.f(x)
    g_x = ROM.g(x)
    grad_k0_uf = ROM_CBF.get_k0_gradient_uf(uf)
    grad_k0_gamma = ROM_CBF.get_k0_derivative_gamma(uf)
    grad_k0_vd = ROM_CBF.get_k0_derivative_vd(uf)

    x_dot = f_x + (g_x @ u_total)

    # FORCE FLATTEN to block NumPy broadcasting traps
    term1 = (grad_k0_x @ x_dot).flatten()
    term2 = (grad_k0_uf @ uf_dot).flatten()
    term3 = (grad_k0_gamma * path.dgamma_dt).flatten()
    term4 = (grad_k0_vd * vd_dot).flatten()

    dt_k0 = term1 + term2 + term3 + term4
    dt_utotal = dt_k0 + uf_dot.flatten()

    # 5. Predict values for time t + dt
    pred_k0 = k0.flatten() + (dt_k0 * dt)
    pred_utotal = u_total.flatten() + (dt_utotal * dt)
    pred_uf = uf.flatten() + (uf_dot.flatten() * dt)

    # 6. Step all systems forward
    uf = input_filter(u_nom, uf, K, dt)
    vd_f = vd_f + vd_dot * dt  # Euler step the path velocity filter forward
    ROM.step(u_total, dt)

    path.update_with_gamma(vd_f)
    path.get_high_order_terms(gamma_dot=vd_f)
    cbf = path.update_barrier_function(cbf, 3, radius=radius)
    ROM_CBF.cbf = cbf

    # 7. Logging
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

print("Simulation Complete. Running Final Diagnostics...\n")


# ==========================================
# POST-SIMULATION DIAGNOSTIC SUITE
# ==========================================

x_test = ROM.get_state().copy()
uf_test = uf.copy()
gamma_test = path.gamma
v_d_test = vd_f  # Using the final filtered state for the diagnostic block

# --- 1. SPATIAL GRADIENT DIAGNOSTIC (grad_k0_x) ---
path.reset(gamma=gamma_test)
path.get_high_order_terms(gamma_dot=v_d_test)
cbf = path.update_barrier_function(cbf, n=3, radius=1)
ROM_CBF.cbf = cbf
ROM.set_control(uf_test)


grad_k0_x_analytical = ROM_CBF.get_sofptlus_gradient()

dx = 1e-5
grad_k0_x_fd = np.zeros((2, 3))

for i in range(3):
    x_plus = x_test.copy()
    x_plus[i] += dx
    ROM.set_state(x_plus)
    k0_plus = ROM_CBF.get_smooth_control().flatten()

    x_minus = x_test.copy()
    x_minus[i] -= dx
    ROM.set_state(x_minus)
    k0_minus = ROM_CBF.get_smooth_control().flatten()

    grad_k0_x_fd[:, i] = (k0_plus - k0_minus) / (2 * dx)

ROM.set_state(x_test)
print("--- 1. SPATIAL GRADIENT DIAGNOSTIC (grad_k0_x) ---")
print("Analytical:\n", np.round(grad_k0_x_analytical, 6))
print("Numerical:\n", np.round(grad_k0_x_fd, 6))
print("Max Difference:", np.max(np.abs(grad_k0_x_analytical - grad_k0_x_fd)))
print("--------------------------------------------------\n")


# --- 2. GAMMA GRADIENT DIAGNOSTIC (grad_k0_gamma) ---
grad_k0_gamma_analytical = ROM_CBF.get_k0_derivative_gamma(uf_test).flatten()

dgamma = 1e-5

path.reset(gamma=gamma_test + dgamma)
path.get_high_order_terms(gamma_dot=v_d_test)
ROM_CBF.cbf = path.update_barrier_function(cbf, n=3, radius=1)
k0_plus = ROM_CBF.get_smooth_control().flatten()

path.reset(gamma=gamma_test - dgamma)
path.get_high_order_terms(gamma_dot=v_d_test)
ROM_CBF.cbf = path.update_barrier_function(cbf, n=3, radius=1)
k0_minus = ROM_CBF.get_smooth_control().flatten()

grad_k0_gamma_fd = (k0_plus - k0_minus) / (2 * dgamma)

path.reset(gamma=gamma_test)
path.get_high_order_terms(gamma_dot=v_d_test)
ROM_CBF.cbf = path.update_barrier_function(cbf, n=3, radius=1)

print("--- 2. GAMMA GRADIENT DIAGNOSTIC (grad_k0_gamma) ---")
print("Analytical:\n", np.round(grad_k0_gamma_analytical, 6))
print("Numerical:\n", np.round(grad_k0_gamma_fd, 6))
print("Max Difference:", np.max(np.abs(grad_k0_gamma_analytical - grad_k0_gamma_fd)))
print("----------------------------------------------------\n")


# --- 3. VELOCITY GRADIENT DIAGNOSTIC (grad_k0_vd) ---
grad_k0_vd_analytical = ROM_CBF.get_k0_derivative_vd(uf_test).flatten()

dv_d = 1e-5

path.get_high_order_terms(gamma_dot=v_d_test + dv_d)
ROM_CBF.cbf = path.update_barrier_function(cbf, n=3, radius=1)
k0_plus_v = ROM_CBF.get_smooth_control().flatten()

path.get_high_order_terms(gamma_dot=v_d_test - dv_d)
ROM_CBF.cbf = path.update_barrier_function(cbf, n=3, radius=1)
k0_minus_v = ROM_CBF.get_smooth_control().flatten()

grad_k0_vd_fd = (k0_plus_v - k0_minus_v) / (2 * dv_d)

path.get_high_order_terms(gamma_dot=v_d_test)
ROM_CBF.cbf = path.update_barrier_function(cbf, n=3, radius=1)

print("--- 3. VELOCITY GRADIENT DIAGNOSTIC (grad_k0_vd) ---")
print("Analytical:\n", np.round(grad_k0_vd_analytical, 6))
print("Numerical:\n", np.round(grad_k0_vd_fd, 6))
print("Max Difference:", np.max(np.abs(grad_k0_vd_analytical - grad_k0_vd_fd)))
print("----------------------------------------------------\n")


# --- 4. FILTERED INPUT GRADIENT DIAGNOSTIC (grad_k0_uf) ---
ROM.set_control(uf_test)
grad_k0_uf_analytical = ROM_CBF.get_k0_gradient_uf(uf_test)

duf = 1e-5
grad_k0_uf_fd = np.zeros((2, 2))

for i in range(2):
    uf_plus = uf_test.copy()
    uf_plus[i] += duf
    ROM.set_control(uf_plus)
    k0_plus = ROM_CBF.get_smooth_control().flatten()

    uf_minus = uf_test.copy()
    uf_minus[i] -= duf
    ROM.set_control(uf_minus)
    k0_minus = ROM_CBF.get_smooth_control().flatten()

    grad_k0_uf_fd[:, i] = (k0_plus - k0_minus) / (2 * duf)

ROM.set_control(uf_test)

print("--- 4. FILTERED INPUT DIAGNOSTIC (grad_k0_uf) ---")
print("Analytical:\n", np.round(grad_k0_uf_analytical, 6))
print("Numerical:\n", np.round(grad_k0_uf_fd, 6))
print("Max Difference:", np.max(np.abs(grad_k0_uf_analytical - grad_k0_uf_fd)))
print("-------------------------------------------------\n")


# --- 5. TOTAL TIME DERIVATIVE DIAGNOSTIC (dt_k0) ---
uf_dot_test = np.array([0.5, -0.2])
vd_dot_test = 1.2
x_dot_test = ROM.f(x_test) + (ROM.g(x_test) @ uf_test)

k0_base = ROM_CBF.get_smooth_control().flatten()

grad_x = ROM_CBF.get_sofptlus_gradient()
grad_uf = ROM_CBF.get_k0_gradient_uf(uf_test)
grad_gamma = ROM_CBF.get_k0_derivative_gamma(uf_test).flatten()
grad_vd = ROM_CBF.get_k0_derivative_vd(uf_test).flatten()

dt_k0_analytical = (grad_x @ x_dot_test).flatten() + \
                   (grad_uf @ uf_dot_test).flatten() + \
                   (grad_gamma * v_d_test) + \
                   (grad_vd * vd_dot_test)

dt_micro = 1e-6

x_micro = x_test + (x_dot_test.flatten() * dt_micro)
uf_micro = uf_test + (uf_dot_test * dt_micro)
gamma_micro = gamma_test + (v_d_test * dt_micro)
v_d_micro = v_d_test + (vd_dot_test * dt_micro)

ROM.set_state(x_micro)
path.reset(gamma=gamma_micro)
path.get_high_order_terms(gamma_dot=v_d_micro)
ROM_CBF.cbf = path.update_barrier_function(cbf, n=3, radius=1)
ROM.set_control(uf_micro)

k0_micro = ROM_CBF.get_smooth_control().flatten()

dt_k0_fd = (k0_micro - k0_base) / dt_micro

ROM.set_state(x_test)
path.reset(gamma=gamma_test)

print("--- 5. TOTAL TIME DERIVATIVE DIAGNOSTIC (dt_k0) ---")
print("Analytical:\n", np.round(dt_k0_analytical, 6))
print("Numerical:\n", np.round(dt_k0_fd, 6))
print("Max Difference:", np.max(np.abs(dt_k0_analytical - dt_k0_fd)))
print("---------------------------------------------------\n")


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

plt.plot(x_log, y_log, label='Robot Path')
cbf_obj = QuadraticBarrier(**cbf_log[0])
cbf_artists = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='blue', linestyle='-')
plt.legend()
plt.show()