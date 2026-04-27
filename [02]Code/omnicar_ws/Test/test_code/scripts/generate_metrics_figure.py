from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt


def generate_metrics_figure(
    metrics: dict,
    info: dict,
    output_path: Path,
    title: str,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 10))
    ax.axis("off")

    lines = [
        title,
        "",
        f"Test ID: {info['test']['test_id']}",
        f"Path: {info['scenario']['path_name']}",
        f"Frame: {info['scenario']['frame']}",
        f"Controller: {info['controller']['controller_name']} {info['controller']['controller_version']}",
        f"Heading mode: {info['controller']['heading_mode']}",
        "",
        "Metrics",
        f"duration_s: {metrics['duration_s']:.4f}",
        f"samples: {metrics['samples']}",
        f"dt_mean_s: {metrics['dt_mean_s']:.4f}",
        f"path_length_real_m: {metrics['path_length_real_m']:.4f}",
        f"gamma_final_m: {metrics['gamma_final_m']:.4f}",
        f"gamma_progress_ratio: {metrics['gamma_progress_ratio']:.4f}",
        f"rmse_x_m: {metrics['rmse_x_m']:.4f}",
        f"rmse_y_m: {metrics['rmse_y_m']:.4f}",
        f"rmse_yaw_rad: {metrics['rmse_yaw_rad']:.4f}",
        f"rmse_error_along_m: {metrics['rmse_error_along_m']:.4f}",
        f"mean_abs_error_along_m: {metrics['mean_abs_error_along_m']:.4f}",
        f"max_abs_error_along_m: {metrics['max_abs_error_along_m']:.4f}",
        f"iae_error_along: {metrics['iae_error_along']:.4f}",
        f"max_abs_error_yaw_rad: {metrics['max_abs_error_yaw_rad']:.4f}",
        f"sat_vx_ratio: {metrics['sat_vx_ratio']:.4f}",
        f"sat_vy_ratio: {metrics['sat_vy_ratio']:.4f}",
        f"sat_w_ratio: {metrics['sat_w_ratio']:.4f}",
    ]

    ax.text(0.02, 0.98, "\n".join(lines), va="top", ha="left", family="monospace", fontsize=11)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
