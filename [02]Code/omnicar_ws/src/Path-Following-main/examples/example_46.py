import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from controllers.cbf_controller import CBFQuadraticQP
from controllers.clf_controllers import CLFCBFQuadraticQP
from dynamic_systems.dynamic_systems import LinearSystem
from functions.barrier_functions import QuadraticBarrier, QuadraticLyapunov

# ==========================================
# ===      EXAMPLE 4.6 SETUP             ===
# ==========================================

# 1. Parameters from Paper (Page 12, Example 4.6)
xc_scalar = 2.0
r = 1.0
p1 = 1.0
p2 = 6.0
p3 = 1.0

# 2. System Matrices (Eq 1502 / 1471)
# A matrix
A = np.array([
    [-p1, 0.0, 0.0],
    [0.0, -p2, p3],
    [0.0, -p3, -p2]
])*1
A_clf = np.array([
    [-p1, 0.0, 0.0],
    [0.0, -p2, p3],
    [0.0, -p3, -p2]
])*(-2)

# A_clf = np.array([
#     [0.5, 0.0, 0.0],
#     [0.0, 1/12,0],
#     [0.0, 0, 1/12]
# ])*(1)

B_clf = np.array([])
# B matrix (Invertible, assumed Identity for G(x)=B^T B to work simply)
B = np.eye(3)

# 3. Initial Condition for Limit Cycle
# The paper derives specific constants p_hat and q_hat for the limit cycle
p_hat = (xc_scalar * p1) / (p2 - p1)  # approx 0.4
q_hat = np.sqrt(r ** 2 - p_hat ** 2)  # approx 0.9165
# Initial state on the cycle
theta = 0 #np.pi + 0*np.pi
x0 = np.array([xc_scalar+ p_hat, q_hat*(np.sin(theta)), q_hat*np.cos(theta)])
#x0 = np.array([3,0,0])
# 4. Instantiate Objects
system = LinearSystem(A,B)
system.set_state(x0)
H = np.eye(3)
p = np.array([xc_scalar,0,0])
cbf = QuadraticBarrier(hessian=H, center=p, height=1,
                       limits=(-1, 1, -1, 1), spacing=0.05, beta1=0.00001)
clf = QuadraticLyapunov(hessian=A_clf, center=p*0, height=0,
                       limits=(-1, 1, -1, 1), spacing=0.05, alpha=0.000001)
controller = CBFQuadraticQP(system, cbf)
clfcbf_controller = CLFCBFQuadraticQP(system, clf=[clf],cbf=[cbf])


# ==========================================
# ===      SIMULATION LOOP               ===
# ==========================================

dt = 0.001
T_max = 5
steps = int(T_max / dt)

x_log, y_log, z_log = [], [], []

for t in range(steps):
    s = system.get_state()

    # Logging
    x_log.append(s[0])
    y_log.append(s[1])
    z_log.append(s[2])

    # Dynamics components
    f_x = (A) @ s
    g_x = B
    print((f_x/np.linalg.norm(f_x)+ clf.gradient(s)/np.linalg.norm(clf.gradient(s)))@cbf.gradient(s)/np.linalg.norm(cbf.gradient(s)))
    # Nominal Controller k(x) = 0 (as specified in Example 4.6)
    u_nom = np.zeros(3)
    #system.set_control(u_nom)

    # Calculate Safety Filter
    #v = clfcbf_controller.get_control()
    v = controller.cbf.cbf_closed_form(s,f_x,g_x,u_nom)
    # Apply Control
    u = u_nom + v
    system.step(u, dt)
    #print(t*dt,s,controller.cbf(s),v)

# ==========================================
# ===      VISUALIZATION (3D)            ===
# ==========================================

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 1. Plot Trajectory
ax.plot(x_log, y_log, z_log, color='orange', linewidth=2, label='Limit Cycle Trajectory')
# Mark start point
ax.scatter([x_log[0]], [y_log[0]], [z_log[0]], color='red', marker='x', s=50, label='Start')

# 2. Plot Unsafe Set (Sphere)
u_ang = np.linspace(0, 2 * np.pi, 50)
v_ang = np.linspace(0, np.pi, 50)
x_s = xc_scalar + r * np.outer(np.cos(u_ang), np.sin(v_ang))
y_s = 0.0 + r * np.outer(np.sin(u_ang), np.sin(v_ang))
z_s = 0.0 + r * np.outer(np.ones(np.size(u_ang)), np.cos(v_ang))
ax.plot_surface(x_s, y_s, z_s, color='green', alpha=0.3)

# 3. Plot Origin
ax.scatter([0], [0], [0], color='black', s=50, label='Origin (Desired Equilibrium)')

# Labels and Styling
ax.set_xlabel('$x_1$')
ax.set_ylabel('$x_2$')
ax.set_zlabel('$x_3$')
ax.set_title(f'Example 4.6: Limit Cycle in 3D\n(No Limit Cycles allowed in 2D, but possible in 3D)')
ax.set_xlim(-1, 4)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)
ax.legend()

plt.show()