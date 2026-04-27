import numpy as np
import yaml
from controllers.path import Path
from functions.barrier_functions import ConcaveQuadraticBarrier

# === 1. Setup ===
with open("../config/config_LCSS.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())

path = Path(conf)
H = np.eye(3)
H[2, 2] = 0
cbf = ConcaveQuadraticBarrier(hessian=H, center=np.zeros(3), height=0.5,
                              limits=(-4, 4, -4, 4), spacing=0.05, beta1=1)

# Freeze state slightly off the path
x = np.array([0.5, 0.5, np.deg2rad(45)])
f_dummy = np.zeros((3, 1))
g_dummy = np.zeros((3, 2))

v_d = 0.008
dgamma = 1e-2
gamma_vals = np.linspace(0.01, 0.99, 10)

print(f"{'Gamma':<8} | {'h_gamma Error':<15} | {'h_gamma_gamma Error':<15}")
print("-" * 45)

for gamma in gamma_vals:
    # --- A. Analytical (Testing your CBF Class) ---
    path.reset(gamma=gamma)

    # Using your realistic velocity!
    path.get_high_order_terms(gamma_dot=v_d)
    cbf = path.update_barrier_function(cbf, n=3, radius=1)

    h_current, _, _, _ = cbf.get_barrier_terms(x, f_dummy, g_dummy)

    # 1. Analytical time derivative (h_dot = v_d * h_gamma)
    h_gamma_analytical = cbf.time_derivative(x, gamma_dot=1.0)


    # 2. Analytical second spatial derivative
    # This formula is pure geometry, so it doesn't need v_d scaling
    h_gamma_gamma_analytical = (((x - cbf.center).T @ cbf.H @ cbf.p_gamma_gamma) - (cbf.p_gamma.T @ cbf.H @ cbf.p_gamma))

    # --- B. Finite Difference (The Ground Truth) ---
    # Step Forward in geometry
    path.reset(gamma=gamma + dgamma)
    path.get_high_order_terms(gamma_dot=v_d)
    cbf_plus = path.update_barrier_function(cbf, n=3, radius=1)
    h_plus, _, _, _ = cbf_plus.get_barrier_terms(x, f_dummy, g_dummy)

    # Step Backward in geometry
    path.reset(gamma=gamma - dgamma)
    path.get_high_order_terms(gamma_dot=v_d)
    cbf_minus = path.update_barrier_function(cbf, n=3, radius=1)
    h_minus, _, _, _ = cbf_minus.get_barrier_terms(x, f_dummy, g_dummy)

    # Numerical Spatial Derivatives
    h_gamma_fd = (h_plus - h_minus) / (2 * dgamma)
    h_gamma_gamma_fd = (h_plus - 2 * h_current + h_minus) / (dgamma ** 2)

    # --- C. Compare ---
    err_1st = np.abs(h_gamma_analytical - h_gamma_fd)
    err_2nd = np.abs(h_gamma_gamma_analytical - h_gamma_gamma_fd)

    print(f"{gamma:<8.2f} | {np.squeeze(err_1st):<15.2e} | {np.squeeze(err_2nd):<15.2e}")