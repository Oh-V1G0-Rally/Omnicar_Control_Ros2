from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def generate_tracking_figure(df: pd.DataFrame, output_path: Path, title: str) -> None:
    t = df["time_from_start_s"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(t, df["target_x"], label="x target", color="tab:blue")
    axes[0].plot(t, df["current_x"], label="x current", color="tab:red", alpha=0.8)
    axes[0].plot(t, df["target_y"], label="y target", color="tab:green")
    axes[0].plot(t, df["current_y"], label="y current", color="tab:orange", alpha=0.8)
    axes[0].set_ylabel("Position [m]")
    axes[0].legend(ncol=2)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, df["target_yaw_unwrapped"], label="yaw target", color="tab:purple")
    axes[1].plot(t, df["current_yaw_unwrapped"], label="yaw current", color="tab:brown", alpha=0.8)
    axes[1].set_ylabel("Yaw [rad]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, df["error_x_world"], label="error_x_world", color="tab:blue")
    axes[2].plot(t, df["error_y_world"], label="error_y_world", color="tab:green")
    axes[2].plot(t, df["error_along"], label="error_along", color="tab:red")
    axes[2].plot(t, df["tracking_error_norm"], label="tracking_error_norm", color="tab:gray")
    axes[2].set_ylabel("Position error [m]")
    axes[2].legend(ncol=2)
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, df["error_yaw_unwrapped"], label="error_yaw", color="tab:purple")
    axes[3].set_ylabel("Yaw error [rad]")
    axes[3].set_xlabel("Time [s]")
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
