import numpy as np
import pandas as pd
from matplotlib import animation, pyplot as plt

from functions.barrier_functions import ConcaveQuadraticBarrier


def animate_cbf_robot(x_log, y_log, theta_log, cbf_log, v_log, w_log, delta_log, dt=0.01, window_size=(10, 10),
                      save_path=None, centerline="../splines/Silverstone/Silverstone_centerline.csv", obs_log=None):
    """
    Animate a robot with CBF lines, control bars, and camera-following view.

    Args:
        x_log, y_log (array): Robot positions over time.
        theta_log (array): Robot orientation over time.
        cbf_log (list of dicts): CBF parameters at each frame.
        v_log, w_log, delta_log (array): Control signals.
        left_x, left_y, right_x, right_y (array): Track boundaries.
        dt (float): Time step between frames in seconds.
        window_size (tuple): Half-width and half-height of camera window.
        save_path (str): File path to save animation.
    """

    # === Load CSV ===
    csv_path = centerline
    df = pd.read_csv(csv_path, comment='#', header=None)
    df.columns = ['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m']

    # === Extract centerline coordinates ===
    x = df['x_m'].values
    y = df['y_m'].values
    w_r = 1#df['w_tr_right_m'].values
    w_l = 1#df['w_tr_left_m'].values

    # === Compute tangents and normals ===
    # Forward difference (approximate tangent)
    dx = np.gradient(x)
    dy = np.gradient(y)
    norms = np.sqrt(dx ** 2 + dy ** 2)
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

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Vehicle Trajectory Animation Circuit of The Americas")
    ax.grid(True)

    # Initialize artists
    robot_dot, = ax.plot([], [], 'ro', markersize=4, label='Robot')
    line, = ax.plot([], [], 'b-', lw=2, label='Path')
    path_preview_line, = ax.plot([], [], 'go', markersize=3, label='Path Preview')  # ← CHANGED HERE
    cbf_lines = []

    # Downsample for performance
    ds = 5
    if obs_log is not None:
        obs_log = obs_log[::ds]
    x_log = x_log[::ds]
    y_log = y_log[::ds]
    theta_log = theta_log[::ds]
    cbf_log = cbf_log[::ds]
    v_log = np.array(v_log[::ds])
    w_log = np.array(w_log[::ds])
    delta_log = np.array(delta_log[::ds])

    max_control = [np.max(np.abs(v_log)), np.max(np.abs(w_log)), np.max(np.abs(delta_log))]

    # Control bars
    bars = [ax.bar(0, 0, width=0.05, color=f'C{i+2}', alpha=0.7, align='edge') for i in range(3)]
    bar_labels = [ax.text(0, 0, label, ha='center', va='top')
                  for label in ['$v$', '$\\omega$', '$v_p$']]

    arrow = None
    window_x, window_y = window_size
    # Plot track boundaries
    ax.plot(left_x, left_y, 'r--')
    ax.plot(right_x, right_y, 'r--')

    def init():
        robot_dot.set_data([], [])
        line.set_data([], [])
        for bar in bars:
            bar[0].set_height(0)
            bar[0].set_y(0)
        path_preview_line.set_data([], [])
        return [robot_dot, line, path_preview_line] + [b[0] for b in bars] + bar_labels

    def update(frame):
        nonlocal arrow

        # Robot position
        x, y = x_log[frame], y_log[frame]
        robot_dot.set_data([x], [y])

        # Local path
        start_idx = max(0, frame - 20)
        line.set_data(x_log[start_idx:frame + 1], y_log[start_idx:frame + 1])

        # Camera follows robot
        ax.set_xlim(x - window_x, x + window_x)
        ax.set_ylim(y - window_y, y + window_y)

        # Clear previous CBF lines
        while cbf_lines:
            cbf_lines.pop().remove()

        # Plot CBF levels
        cbf_obj = ConcaveQuadraticBarrier(**cbf_log[frame])
        new_lines = cbf_obj.plot_levels(ax=ax, levels=[0.0], color='green', linestyle='-')
        cbf_lines.extend(new_lines)

        # Orientation arrow
        if arrow is not None:
            arrow.remove()
        theta = theta_log[frame]
        dx = np.cos(theta) * 0.5
        dy = np.sin(theta) * 0.5
        arrow = ax.arrow(x, y, dx, dy, head_width=0.3, head_length=0.4, fc='k', ec='k')

        # Update control bars
        ax_xlim = ax.get_xlim()
        ax_ylim = ax.get_ylim()
        x0 = ax_xlim[0] + 0.75 * (ax_xlim[1] - ax_xlim[0])
        y0 = ax_ylim[0] + 0.15 * (ax_ylim[1] - ax_ylim[0])
        controls = [v_log[frame], w_log[frame], delta_log[frame]]
        for i, bar in enumerate(bars):
            value = controls[i] / max_control[i]
            bar_height = value * 0.15 * (ax_ylim[1] - ax_ylim[0])
            bar[0].set_height(abs(bar_height))
            bar[0].set_y(y0 if value >= 0 else y0 + bar_height)
            bar[0].set_x(x0 + i * 0.08 * (ax_xlim[1] - ax_xlim[0]))
            bar[0].set_width(0.04*(ax_xlim[1] - ax_xlim[0]))
            bar_labels[i].set_position(
                (x0 + i * 0.08 * (ax_xlim[1] - ax_xlim[0]) + 0.025 * (ax_xlim[1] - ax_xlim[0]),
                 y0 - 0.02 * (ax_ylim[1] - ax_ylim[0])))

            # Optional path preview from observations
            if obs_log is not None:
                obs = obs_log[frame]
                path_robot = obs[2:80]  # Nx2
                # Compute robot heading
                theta = theta_log[frame]
                R = np.array([[np.cos(theta), -np.sin(theta)],
                              [np.sin(theta), np.cos(theta)]])
                path_world = path_robot @ R.T + np.array([x_log[frame], y_log[frame]])
                path_preview_line.set_data(path_world[:, 0], path_world[:, 1])
            else:
                path_preview_line.set_data([], [])

        return [robot_dot, line, arrow, path_preview_line] + cbf_lines + [b[0] for b in bars] + bar_labels

    anim = animation.FuncAnimation(
        fig=fig,
        func=update,
        frames=len(cbf_log),
        init_func=init,
        interval=dt,
    )
    #
    if save_path is not None:
        anim.save(save_path, writer='ffmpeg', fps=30, dpi=200)
    else:
        plt.show()
    plt.close(fig)





import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_trajectories(x_log, y_log,v_log, centerline="../splines/Silverstone/Silverstone_centerline.csv"):
    """
    Plot robot trajectories over the track centerline and boundaries.

    Args:
        x_log, y_log (array-like): Robot positions over time.
        save_path (str, optional): File path to save the figure.
        centerline (str): Path to CSV containing centerline and width info.
    """

    # === Load CSV ===
    plt.rcParams.update({'font.size': 16})
    df = pd.read_csv(centerline, comment='#', header=None)
    df.columns = ['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m']

    # === Extract centerline coordinates ===
    x = df['x_m'].values
    y = df['y_m'].values
    w_r = df['w_tr_right_m'].values
    w_l = df['w_tr_left_m'].values

    # === Compute tangents and normals ===
    dx = np.gradient(x)
    dy = np.gradient(y)
    norms = np.sqrt(dx ** 2 + dy ** 2)
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

    # === Plot ===
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_aspect('equal')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Vehicle Trajectory in the Brands Hatch Track")

    # Plot track
    ax.plot(-1*np.array(left_y), left_x, 'r--', label='Track Limits')
    ax.plot(-1*np.array(right_y), right_x, 'r--')
    #ax.plot(x, y, 'k-', linewidth=1, label='Centerline')

    # Plot trajectory
    ax.plot(-1*np.array(y_log), x_log, 'b-', linewidth=2, label='Vehicle trajectory')
    ax.scatter(x_log[0], y_log[0], c='blue', marker='o', s=50, label='Start')

    ax.legend()
    ax.grid(True)
    plt.show()


import numpy as np
import matplotlib.pyplot as plt

def plot_controls(v_log, w_log,vd_log, t_log, save_path=None):
    """
    Plot linear (v) and angular (w) velocities over time on a single y-axis.

    Args:
        v_log (array-like): Linear velocity log [m/s].
        w_log (array-like): Angular velocity log [rad/s].
        t_log (array-like): Time log [s].
        save_path (str, optional): Path to save the figure.
    """
    v_log = np.asarray(v_log)
    w_log = np.asarray(w_log)
    t_log = np.asarray(t_log)
    # === Load CSV ===
    plt.rcParams.update({'font.size': 16})
    plt.figure(figsize=(10, 6))
    plt.plot(t_log, vd_log, color='tab:green', linewidth=2, label='Path velocity $v_p$ [m/s]')
    plt.plot(t_log, v_log, color='tab:blue', linewidth=2, label='Linear velocity $v$ [m/s]')
    plt.plot(t_log, w_log, color='tab:orange', linewidth=2, label='Angular velocity $\\omega$ [rad/s]')

    plt.xlabel("Time [s]")
    plt.ylabel("Velocity")
    plt.title("State Variables Over Time (Silverstone)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


    plt.show()

