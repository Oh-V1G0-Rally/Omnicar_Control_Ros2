import time

from Robot import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import yaml
import matplotlib.animation as animation
import pandas as pd
from Robot.DiffDrive import *
from controllers.path import *
from dynamic_systems.systems import *
csv_path = "../splines/Silverstone/Silverstone_centerline.csv"  # path to your CSV
df = pd.read_csv(csv_path, comment='#', header=None)
df.columns = ['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m']

# === Extract centerline coordinates ===
x = df['x_m'].values
y = df['y_m'].values
w_r = df['w_tr_right_m'].values
w_l = df['w_tr_left_m'].values

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

with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file

dt = conf["dt_control"]
np.random.seed(1111)
rob = Unicycle()
x, y, t_log, theta, w_log, wref_log, v_log, vref_log = [], [], [], [], [], [], [], []
start_time = time.time()
PF = Path(conf)
map =PF.tck[1]
state = rob.get_state()
t=0
for _ in range(int(50 * 1 / dt)):
    PF.get_high_order_terms(10)
    PF.update_with_vref(10)
    u = PF.follow_path(state[0],state[1],state[2])
    #u = np.clip(u,[-10,-2],[10,2])
    rob.step(u,dt)

    # State: [x, y, theta]
    state = rob.get_state()
    x.append(state[0])
    y.append(state[1])
    theta.append(state[2])

    # Control input to robot (v, w)

    #v_log.append(state[3])
    #w_log.append(state[4])
    v_log.append(u[0])
    w_log.append(u[1])
    # Reference values (computed in rob.ref)
    vref_log.append(u[0])
    wref_log.append(u[1])

    # Time
    t_log.append(t)
    t += dt
    if PF.gamma >0.99:
        break

    #print(rob.x,rob.w)
print(t)
end_time = time.time()

# Print elapsed time
print(f"Simulation loop completed in {end_time - start_time:.4f} seconds")

# === Plot ===


plt.plot(left_x, left_y, 'b--', label='Left boundary')
plt.plot(right_x, right_y, 'r--', label='Right boundary')


plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Track Centerline and Boundaries")
plt.legend()


plt.plot(x, y)
plt.plot(map[0],map[1])
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Path')
plt.show()

plt.plot(t_log,v_log,label="velocity")
plt.plot(t_log,vref_log,label="reference")
plt.legend()
plt.show()
# === Sample data from your simulation (no renaming needed) ===
sample_rate = 5
x_sampled = x[::sample_rate]
y_sampled = y[::sample_rate]
theta_sampled = theta[::sample_rate]
t_sampled = t_log[::sample_rate]

# === Robot dimensions (based on DiffDrive size) ===
robot_length = 0.3
robot_width = 0.2

# === Set up figure and static elements (your original plot) ===
fig, ax = plt.subplots(figsize=(10, 10))
ax.plot(left_x, left_y, 'b--', label='Left boundary')
ax.plot(right_x, right_y, 'r--', label='Right boundary')
ax.set_aspect('equal')
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Robot Path on Silverstone Track")
ax.legend()
ax.grid(True)

# === Robot patch and trail ===
robot_rect = patches.Rectangle((0, 0), robot_length, robot_width, fc='red', zorder=5)
ax.add_patch(robot_rect)
trail, = ax.plot([], [], 'g-', linewidth=1.5, label='Robot Path')

# === Animation init ===
def init():
    robot_rect.set_xy((-1, -1))  # off-screen
    trail.set_data([], [])
    return robot_rect, trail

# === Animation update ===
def animate(i):
    x_ = x_sampled[i]
    y_ = y_sampled[i]
    theta_ = theta_sampled[i]

    # Update robot pose
    transform = patches.transforms.Affine2D().rotate_around(x_, y_, theta_) + ax.transData
    robot_rect.set_xy((x_ - robot_length/2, y_ - robot_width/2))
    robot_rect.set_transform(transform)

    # Update trail
    trail.set_data(x_sampled[:i+1], y_sampled[:i+1])
    return robot_rect, trail

# === Run animation ===
ani = animation.FuncAnimation(
    fig, animate, init_func=init,
    frames=len(x_sampled), interval=dt * 50 * sample_rate,
    blit=True
)

plt.show()
#ani.save("robot_animation.mp4", writer='ffmpeg')