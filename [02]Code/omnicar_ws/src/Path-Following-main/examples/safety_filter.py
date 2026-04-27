from platform import system
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy

from numpy.ma.core import masked_outside

# Assuming these imports work in your local environment
from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from controllers.clf_controllers import get_tangent_vector, CLFCBFQuadraticQP
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
from optimization_programs.equilibria_finder import EquilibriaFinder

# === System setup ===
system = Integrator(n=2)
# Define Lyapunov Function
H = np.diag([1, 1])
xd = np.array([0, 0])
clf = QuadraticLyapunov(hessian=H, center=xd, height=0,
                        limits=(-4, 4, -4, 4), spacing=0.05, alpha=0.1)

dt = 0.005
# Define Barrier Function
H = canonical2D([3, 20], np.rad2deg(0.0))
p = np.array([0, -0.5])
cbf = QuadraticBarrier(hessian=H, center=p, height=1,
                       limits=(-1, 1, -1, 1), spacing=0.05, beta1=100)

initial_condition= np.array([-0.8,-0.8])
cbf_log = []
controller = CLFCBFQuadraticQP(system,[clf],[cbf])

# === Data storage ===
x_log = []
y_log = []
goalx_log = []
goaly_log = []

# Initial State
system.set_state(initial_condition)
goal = np.array([0, 0])

# Find Equilibria (Roots)
solver = EquilibriaFinder(clf=clf, cbf=cbf, plant=system)
roots = solver.solve()
roots = solver.filter_non_repulsive_equilibria(roots)
if len(roots) >= 3:
    roots = solver.find_splitting_pair(roots[0],roots[1],roots[2])
    print(roots)

# === Simulation Loop ===
for _ in range(int(4 * 1 / dt)):
    s = system.get_state()
    goal = np.array([0.0, 0.0])
    u_nom = 2 * (goal - s)

    # Check Active Constraints to switch goals (simple logic)
    system.set_control(u_nom)
    #active = controller.find_active_constraints()
    # if active:
    #    goal = np.array([0.5, 0])
    #    u_nom = 2 * (goal - s)

    system.set_control(u_nom)
    u = controller.get_control()
    print(u_nom,u)
    system.step(u_nom + u, dt)
    s = system.get_state()

    # Log data
    x_log.append(s[0])
    y_log.append(s[1])
    goalx_log.append(goal[0])
    goaly_log.append(goal[1])
    cbf_log.append({
        "hessian": copy.deepcopy(cbf.H),
        "center": copy.deepcopy(cbf.center),
        "height": copy.deepcopy(cbf.height),
        "limits": (cbf.center[0] - 3, cbf.center[0] + 3, cbf.center[1] - 3, cbf.center[1] + 3),
        "spacing": cbf.spacing
    })

# === Downsample ===
ds = 5
x_log = x_log[::ds]
y_log = y_log[::ds]
goalx_log = goalx_log[::ds]
goaly_log = goaly_log[::ds]
cbf_log = cbf_log[::ds]

# === Animation setup ===
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_aspect('equal')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Purple: Safe Cone | Cyan: Restoration Flow")
ax.grid(True)

line, = ax.plot([], [], 'b-', lw=2, label='Path')
robot_dot, = ax.plot([], [], 'ro', markersize=5, label='Robot')
goal_dot, = ax.plot([], [], 'yo', markersize=5, label='Goal')

# Plot Equilibrium Points
roots = np.array(roots)
if len(roots) > 0:
    plt.scatter(roots[:, 0], roots[:, 1], color='brown', zorder=5, label='Equilibria')

# Plot CBF Zero Level Set
cbf_obj = ConcaveQuadraticBarrier(**cbf_log[0])
cbf_obj.plot_levels(ax=ax, levels=[0.0], color='green', linestyle='-')

# ==========================================
# ===      REGION VISUALIZATION          ===
# ==========================================

# 1. Geometry Setup (Roots & Reference)
line_a = roots[2]  # Root 1
line_b = roots[1]  # Root 2
dir_p = roots[0]   # Reference point (inside safe set)

# 2. Create Mesh Grid
gx = np.linspace(-2.0, 2.0, 500)
gy = np.linspace(-2.0, 2.0, 500)
GX, GY = np.meshgrid(gx, gy)
points_flat = np.column_stack([GX.ravel(), GY.ravel()])

# 3. Mask A: "Base" (Above the line connecting the two roots)
def get_orientation_vectorized(a, b, px, py):
    return (b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0])

base_ref = get_orientation_vectorized(line_a, line_b, dir_p[0], dir_p[1])
base_grid = get_orientation_vectorized(line_a, line_b, GX, GY)
mask_base = (base_ref * base_grid) >= 0

# 4. Mask B: "CBF Active" (Controller is modifying input)
A_mat = cbf.A
b_vec = cbf.b
c_val = cbf.c
beta = cbf.beta1
goal_origin = np.array([0, 0])

# h(x)
term_quad = np.einsum('ij, jk, ik -> i', points_flat, A_mat, points_flat)
term_lin = points_flat @ b_vec
h_vals = term_quad + term_lin + c_val

# grad(h) / Lgh
grad_vals = points_flat @ (A_mat + A_mat.T).T + b_vec
# Note: For Integrator system, Lgh = grad(h)

# u_nom (nominal control toward origin)
u_nom_vals = 2 * (goal_origin - points_flat)

# Lie Derivative (LgV * u_nom)
# FIX 1: Use row-wise dot product (sum(a*b)), not matrix mult (@)
lf_h_vals = np.sum(grad_vals * u_nom_vals, axis=1)

# Constraint Check
cbf_condition = lf_h_vals + (beta * h_vals)
mask_cbf_active = (cbf_condition <= 0).reshape(GX.shape)

# 5. Mask C: "Geometric Cone" (Inside Tangent Planes)
grad_a = cbf.gradient(line_a)
grad_b = cbf.gradient(line_b)
top = line_a -line_b
top_tangent = get_tangent_vector(top)

# Plane checks
mask_plane_a = (np.dot(points_flat - line_a, get_tangent_vector(grad_a)) >= 0).reshape(GX.shape)
mask_plane_b = (np.dot(points_flat - line_b, get_tangent_vector(grad_b)) <= 0).reshape(GX.shape)
mask_plane_c = (np.dot(points_flat - line_a, top_tangent) <= 0).reshape(GX.shape)

mask_inside_cone =  mask_plane_c & mask_plane_a & mask_plane_b #mask_plane_c #& mask_plane_a & mask_plane_b
mask_outside_cone = ~mask_inside_cone

# 6. Mask D: "Flow Alignment" (Restoration)
# FIX 2: Calculate 'flow' exactly as requested (The Correction Vector)
denom = np.sum(grad_vals ** 2, axis=1)
denom[denom < 1e-6] = 1e-6

psi = lf_h_vals + beta * h_vals
lambda_vals = psi / denom
flow_vectors = u_nom_vals-grad_vals * lambda_vals[:, np.newaxis]

# --- FIX START: ROBUST INWARD CHECK ---

# A. Get Normals (Perpendicular to the visual walls)
# Since visual walls are along grad_a/b, the normals are the tangents.
n_a = get_tangent_vector(grad_a)
n_b = get_tangent_vector(grad_b)

# B. Orient Normals to point INWARDS (towards dir_p)
# This auto-corrects signs so you don't have to guess >= or <=
if np.dot(n_a, dir_p - line_a) < 0: n_a = -n_a
if np.dot(n_b, dir_p - line_b) < 0: n_b = -n_b
if np.dot(top_tangent, dir_p - (line_a-line_b)/2) < 0: top_tangent = -top_tangent

# C. Check Alignment
# Flow is "inwards" if it pushes positively along BOTH inward normals
dot_flow_a = flow_vectors @ n_a
dot_flow_b = flow_vectors @ n_b
dot_flow_c = flow_vectors @ top_tangent


initial_condition= np.array([0.4,-0.9])
goal = np.array([0,0])
test = 2*(goal-initial_condition)#+controller.cbf.cbf_closed_form(initial_condition,np.zeros(2),np.eye(2),u_nom= 2*(goal-initial_condition))
print(np.dot(test,n_a), np.dot(test,top_tangent))

mask_flow_inwards = ((dot_flow_a >=0) | (dot_flow_b >= 0)  ).reshape(GX.shape)
mask_flow_inwards &= mask_plane_c
#mask_flow_inwards = ((dot_flow_a <= 0)).reshape(GX.shape)


# ------------------------------------------
# COMBINE AND PLOT
# ------------------------------------------
# Original Purple: Active & Above Base & Inside Cone
mask_purple =  mask_base & mask_inside_cone

# New Cyan: Active & Outside Cone & Flow Points In
mask_cyan =  mask_cbf_active & mask_base #&  mask_flow_inwards

# Plotting
ax.contourf(GX, GY, mask_purple, levels=[0.5, 1.5], colors=['purple'], alpha=0.3)
ax.contourf(GX, GY, mask_cyan, levels=[0.5, 1.5], colors=['cyan'], alpha=0.5)

# Visualizing the walls
ax.plot([line_a[0], line_b[0]], [line_a[1], line_b[1]], 'r-', lw=2, label='Base Wall')
scale = 1.0
ax.plot([line_a[0], line_a[0]+grad_a[0]*scale], [line_a[1], line_a[1]+grad_a[1]*scale], 'k--', lw=1, label='Side Walls')
ax.plot([line_b[0], line_b[0]+grad_b[0]*scale], [line_b[1], line_b[1]+grad_b[1]*scale], 'k--', lw=1)

ax.legend(loc='upper right')

def init():
    line.set_data([], [])
    robot_dot.set_data([], [])
    goal_dot.set_data([], [])
    return [line, robot_dot, goal_dot]


def update(frame):
    line.set_data(x_log[:frame + 1], y_log[:frame + 1])
    robot_dot.set_data([x_log[frame]], [y_log[frame]])
    goal_dot.set_data([goalx_log[frame]], [goaly_log[frame]])
    return [line, robot_dot, goal_dot]


ani = animation.FuncAnimation(
    fig=fig, func=update, frames=len(cbf_log), init_func=init, interval=20,
)

plt.show()