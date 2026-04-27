from platform import system
import numpy as np
import matplotlib.pyplot as plt
import copy

from sympy import true

# Assuming these imports work in your local environment
from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from controllers.clf_controllers import get_tangent_vector
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
from optimization_programs.equilibria_finder import EquilibriaFinder

# === System setup ===
system = Integrator(n=2)
H = np.diag([1, 1])
xd = np.array([0, 0])
clf = QuadraticLyapunov(hessian=H, center=xd, height=0,
                        limits=(-4, 4, -4, 4), spacing=0.05, alpha=0.5)

H_cbf = canonical2D([5, 20], np.rad2deg(0))
p = np.array([0, -0.5])
cbf = QuadraticBarrier(hessian=H_cbf, center=p, height=1,
                       limits=(-1, 1, -1, 1), spacing=0.05, beta1=10)

solver = EquilibriaFinder(clf=clf, cbf=cbf, plant=system)
roots = solver.solve()
roots = solver.filter_non_repulsive_equilibria(roots)

# === SETUP SUBPLOTS ===
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 12), height_ratios=[3, 1])

# ==========================================
# ===      TOP PLOT: MAP & FLOW          ===
# ==========================================

ax1.set_aspect('equal')
ax1.set_xlim(-1, 1)
ax1.set_ylim(-2, 0.1)
ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.set_title("Total Safe Flow Field")
ax1.grid(True)

# CBF Level Set
cbf_log_entry = {
    "hessian": copy.deepcopy(cbf.H),
    "center": copy.deepcopy(cbf.center),
    "height": copy.deepcopy(cbf.height),
    "limits": (cbf.center[0] - 3, cbf.center[0] + 3, cbf.center[1] - 3, cbf.center[1] + 3),
    "spacing": cbf.spacing
}
cbf_obj = ConcaveQuadraticBarrier(**cbf_log_entry)
cbf_obj.plot_levels(ax=ax1, levels=[0.0], color='green', linestyle='-', linewidth=2)

# Equilibria
roots = np.array(roots)
if len(roots) > 0:
    ax1.scatter(roots[:, 0], roots[:, 1], color='brown', zorder=5, label='Equilibria', s=50)

# --- 1. Geometry & Masks ---
line_a = roots[0]; line_b = roots[1]; dir_p = roots[2]
gx = np.linspace(-1.0, 1.0, 200)
gy = np.linspace(-2.0, 0.1, 200)
GX, GY = np.meshgrid(gx, gy)
points_flat = np.column_stack([GX.ravel(), GY.ravel()])

# Base Mask
def get_orientation_vectorized(a, b, px, py):
    return (b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0])
base_ref = get_orientation_vectorized(line_a, line_b, dir_p[0], dir_p[1])
base_grid = get_orientation_vectorized(line_a, line_b, GX, GY)
mask_base = (base_ref * base_grid) >= 0

# Terms
A_mat = cbf.A; b_vec = cbf.b; c_val = cbf.c; beta = cbf.beta1
goal_origin = np.array([0, 0])

term_quad = np.einsum('ij, jk, ik -> i', points_flat, A_mat, points_flat)
term_lin = points_flat @ b_vec
h_vals = term_quad + term_lin + c_val
grad_vals = points_flat @ (A_mat + A_mat.T).T + b_vec
u_nom_vals = 2 * (goal_origin - points_flat)
lf_h_vals = np.sum(grad_vals * u_nom_vals, axis=1)
cbf_condition = lf_h_vals + (beta * h_vals)
mask_cbf_active = (cbf_condition <= 0).reshape(GX.shape)

# Cone Masks (Using Normals perpendicular to Gradient to match dashed lines)
grad_a = cbf.gradient(line_a); grad_b = cbf.gradient(line_b)
n_a = np.array([-grad_a[1], grad_a[0]]) # Rotated 90
n_b = np.array([-grad_b[1], grad_b[0]])

if np.dot(n_a, dir_p - line_a) < 0: n_a = -n_a
if np.dot(n_b, dir_p - line_b) < 0: n_b = -n_b

mask_plane_a = (np.dot(points_flat - line_a, n_a) >= 0).reshape(GX.shape)
mask_plane_b = (np.dot(points_flat - line_b, n_b) >= 0).reshape(GX.shape)
mask_inside_cone = mask_plane_a & mask_plane_b
mask_outside_cone = ~mask_inside_cone

# Flow Calculation
denom = np.sum(grad_vals ** 2, axis=1)
denom[denom < 1e-6] = 1e-6
psi = lf_h_vals + beta * h_vals
lambda_vals = psi / denom
is_safe = (psi > 0)
lambda_vals[is_safe] = 0.0

flow_vectors_flat =  grad_vals * lambda_vals[:, np.newaxis]

# Inward Check
# We use the same normals n_a/n_b used for the cone
dot_flow_a = flow_vectors_flat @ n_a
dot_flow_b = flow_vectors_flat @ n_b
mask_flow_inwards = ((dot_flow_a >= 0) & (dot_flow_b >= 0)).reshape(GX.shape)

mask_purple = mask_cbf_active #& mask_base & mask_inside_cone
#mask_cyan = mask_flow_inwards

# Plot Regions
ax1.contourf(GX, GY, mask_purple, levels=[0.5, 1.5], colors=['purple'], alpha=0.3)
#ax1.contourf(GX, GY, mask_cyan, levels=[0.5, 1.5], colors=['cyan'], alpha=0.3)

# Plot Walls
ax1.plot([line_a[0], line_b[0]], [line_a[1], line_b[1]], 'r-', lw=2, label='Base Wall')
scale = 1.0
ax1.plot([line_a[0], line_a[0]+grad_a[0]*scale], [line_a[1], line_a[1]+grad_a[1]*scale], 'k--', lw=1, label='Side Walls (Grad)')
ax1.plot([line_b[0], line_b[0]+grad_b[0]*scale], [line_b[1], line_b[1]+grad_b[1]*scale], 'k--', lw=1)

# Plot Quiver
FlowX = flow_vectors_flat[:, 0].reshape(GX.shape)
FlowY = flow_vectors_flat[:, 1].reshape(GX.shape)
M = np.hypot(FlowX, FlowY); M[M == 0] = 1.0
FlowX /= M; FlowY /= M
skip = 5
ax1.quiver(GX[::skip, ::skip], GY[::skip, ::skip], FlowX[::skip, ::skip], FlowY[::skip, ::skip],
           color='black', scale=35, width=0.003, alpha=0.6)

# Line Indication on Top Plot
y_fixed = -2/3
ax1.axhline(y_fixed, color='orange', linestyle='-.', linewidth=2, label=f'Slice y={y_fixed}')
ax1.legend(loc='upper right')

# ==========================================
# ===      BOTTOM PLOT: ANGLE PROFILE    ===
# ==========================================

# 1. Profile Calculation
x_vals = np.linspace(-1.5, 1.5, 500)
y_col = np.full_like(x_vals, y_fixed)
points_slice = np.column_stack([x_vals, y_col])

grad_s = points_slice @ (A_mat + A_mat.T).T + b_vec
u_nom_s = np.zeros_like(points_slice)
u_nom_s[:, 0] = -2.0 * (goal_origin[0] - points_slice[:, 0])
u_nom_s[:, 1] = -2.0 * (goal_origin[1] - points_slice[:, 1])# h_s = np.einsum('ij, jk, ik -> i', points_slice, A_mat, points_slice) + points_slice @ b_vec + c_val
# denom_s = np.sum(grad_s ** 2, axis=1); denom_s[denom_s < 1e-6] = 1e-6
# psi_s = lf_h_s + beta * h_s
# lambda_s = psi_s / denom_s
# lambda_s[True] = 0.0

#flow_s = u_nom_s - grad_s * lambda_s[:, np.newaxis]
u_nom_deg = np.degrees(-np.arctan2(u_nom_s[:,1], u_nom_s[:,0]))
grad_cbf_deg = np.degrees(-np.arctan2(grad_s[:,1], grad_s[:,0]))
#angles_deg = np.degrees(np.arctan2(flow_s[:, 1], flow_s[:, 0]))

# 2. Wall Orientation Calculation (Gradient Direction)
# The side walls in the top plot are drawn along the Gradient Vectors (grad_a, grad_b)
# So we calculate the angle of these specific vectors.
angle_grad_a = np.degrees(np.arctan2(-grad_a[1], -grad_a[0])) # Negate to match visual "outward/down" dashed line
angle_grad_b = np.degrees(np.arctan2(-grad_b[1], -grad_b[0]))

# 3. Plotting
#ax2.plot(x_vals, angles_deg, linewidth=2, color='blue', label='Flow Orientation')
ax2.plot(x_vals, u_nom_deg, linewidth=2, color='blue', label='cbf grad Orientation')
ax2.set_xlabel("X Coordinate (m)")
ax2.set_ylabel("Angle (deg)")
ax2.set_title(f"Flow Angle vs X (at y={y_fixed})")
ax2.grid(True)
ax2.set_ylim(-180, 180)
ax2.set_xlim(-1.5, 1.5)
ax2.set_yticks(np.arange(-180, 181, 45))

# Add Wall Orientations
ax2.axhline(angle_grad_a, color='black', linestyle='--', linewidth=1.5, label=f'Left Wall Grad ({angle_grad_a:.1f}°)')
ax2.axhline(angle_grad_b, color='black', linestyle='--', linewidth=1.5, label=f'Right Wall Grad ({angle_grad_b:.1f}°)')

ax2.legend(loc='lower right')
plt.tight_layout()
plt.show()