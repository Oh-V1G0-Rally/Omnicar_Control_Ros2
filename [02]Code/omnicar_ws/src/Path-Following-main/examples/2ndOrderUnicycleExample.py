import yaml

from dynamic_systems.systems import *
import matplotlib.pyplot as plt
system = Unicycle2ndOrder()

with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())

dt = conf["dt_control"]


# === Data storage ===
x_log = []
y_log = []
theta_log = []
v_log = []
w_log = []

# === Simulation ===
for _ in range(int(1 * 1 / dt)):  # 1 second sim
    u = [1, 1]  # accel forward, no angular accel
    system.step(u, dt)
    state = system.get_state()
    x_log.append(state[0])
    y_log.append(state[1])
    theta_log.append(state[2])
    v_log.append(state[3])
    w_log.append(state[4])

# === Plot results ===
time = [i * dt for i in range(len(v_log))]

plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(time, x_log, label="x")
plt.plot(time, y_log, label="y")
plt.legend()
plt.ylabel("Position [m]")

plt.subplot(3, 1, 2)
plt.plot(time, theta_log, label="theta")
plt.legend()
plt.ylabel("Heading [rad]")

plt.subplot(3, 1, 3)
plt.plot(time, v_log, label="v")
plt.plot(time, w_log, label="w")
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Velocities")

plt.tight_layout()
plt.show()