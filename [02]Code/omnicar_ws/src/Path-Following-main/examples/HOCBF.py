import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from controllers.cbf_controller import *
from dynamic_systems.systems import Unicycle2ndOrder, Bicycle, BicycleInnerLoop
from functions.barrier_functions import canonical2D, ConcaveQuadraticBarrier

# === System initialization ===
state = np.array([-3, 1.5, 0, 1.57])
system = BicycleInnerLoop(state=state)

with open("../config/config.yaml", "rb") as f:
    conf = yaml.safe_load(f.read())

dt = conf["dt_control"]

H_ = canonical2D([1, 1], np.rad2deg(0.0))
H = np.zeros((4, 4))
H[0:2, 0:2] = H_
p = np.array([0, 0, 0, 0])

cbf = ConcaveQuadraticBarrier(hessian=H, center=p, limits=(-4, 4, -4, 4), spacing=0.05, beta1=100,beta2=100)
controller = HOCBFQuadraticQP(system, cbf)

# === Data storage ===
x_log = []
y_log = []
theta_log = []
v_log = []
w_log = []
# === Simulation ===
for _ in range(int(5 * 1 / dt)):  # 10 second sim

    u = controller.get_control()
    print(u)
    system.step(u, dt)
    s = system.get_state()
    x_log.append(s[0])
    y_log.append(s[1])
    theta_log.append(s[3])
    v_log.append(s[2])
    w_log.append(u[1])


# === Time axis ===
time = [i * dt for i in range(len(x_log))]

# === Plot results ===
plt.figure(figsize=(10, 8))

# 2D trajectory
plt.subplot(2, 2, 1)
plt.plot(x_log, y_log, label="trajectory")
plt.scatter(x_log[0], y_log[0], c="green", marker="o", label="start")
plt.scatter(x_log[-1], y_log[-1], c="red", marker="x", label="end")

# Add unit circle centered at origin
circle = patches.Circle((0, 0), radius=1, fill=False, color="blue", linestyle="--", label="safe set boundary")
plt.gca().add_patch(circle)

plt.axis("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("2D Trajectory")
plt.legend()

# Theta
plt.subplot(2, 2, 2)
plt.plot(time, theta_log, label="theta")
plt.xlabel("Time [s]")
plt.ylabel("Heading [rad]")
plt.title("Heading")
plt.legend()

# Linear velocity
plt.subplot(2, 2, 3)
plt.plot(time, v_log, label="v")
plt.xlabel("Time [s]")
plt.ylabel("Linear Velocity [m/s]")
plt.title("Linear Velocity")
plt.legend()

# Angular velocity
plt.subplot(2, 2, 4)
plt.plot(time, w_log, label="w")
plt.xlabel("Time [s]")
plt.ylabel("Angular Velocity [rad/s]")
plt.title("Angular Velocity")
plt.legend()

plt.tight_layout()
plt.show()