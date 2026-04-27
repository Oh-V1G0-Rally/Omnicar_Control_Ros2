from __future__ import annotations

import numpy as np
import pandas as pd

from common import as_bool_ratio, mae, max_abs, rmse


def compute_metrics(df: pd.DataFrame, path_reference: pd.DataFrame | None = None) -> dict[str, float]:
    if df.empty:
        raise ValueError("Cannot compute metrics from an empty dataframe")

    duration_s = float(df["time_from_start_s"].iloc[-1] - df["time_from_start_s"].iloc[0])
    dt_mean_s = float(df["time_from_start_s"].diff().dropna().mean()) if len(df) > 1 else 0.0

    dx = pd.to_numeric(df["current_x"], errors="coerce").diff().fillna(0.0)
    dy = pd.to_numeric(df["current_y"], errors="coerce").diff().fillna(0.0)
    path_length_real_m = float(np.hypot(dx, dy).sum())

    gamma_final_m = float(pd.to_numeric(df["gamma"], errors="coerce").iloc[-1])
    if path_reference is not None and not path_reference.empty:
        gamma_total = float(pd.to_numeric(path_reference["gamma"], errors="coerce").iloc[-1])
    else:
        gamma_total = float(pd.to_numeric(df["gamma"], errors="coerce").max())
    gamma_progress_ratio = gamma_final_m / gamma_total if gamma_total > 0.0 else 0.0

    metrics = {
        "duration_s": duration_s,
        "samples": int(len(df)),
        "dt_mean_s": dt_mean_s,
        "path_length_real_m": path_length_real_m,
        "gamma_final_m": gamma_final_m,
        "gamma_progress_ratio": gamma_progress_ratio,
        "rmse_x_m": rmse(pd.to_numeric(df["current_x"], errors="coerce") - pd.to_numeric(df["target_x"], errors="coerce")),
        "rmse_y_m": rmse(pd.to_numeric(df["current_y"], errors="coerce") - pd.to_numeric(df["target_y"], errors="coerce")),
        "rmse_yaw_rad": rmse(df["error_yaw_unwrapped"]),
        "rmse_error_along_m": rmse(df["error_along"]),
        "mean_abs_error_along_m": mae(df["error_along"]),
        "max_abs_error_along_m": max_abs(df["error_along"]),
        "iae_error_along": float(
            np.trapz(
                np.abs(pd.to_numeric(df["error_along"], errors="coerce").fillna(0.0).to_numpy()),
                pd.to_numeric(df["time_from_start_s"], errors="coerce").fillna(0.0).to_numpy(),
            )
        ),
        "max_abs_error_yaw_rad": max_abs(df["error_yaw_unwrapped"]),
        "sat_vx_ratio": as_bool_ratio(df["saturated_vx"]),
        "sat_vy_ratio": as_bool_ratio(df["saturated_vy"]),
        "sat_w_ratio": as_bool_ratio(df["saturated_w"]),
    }
    return metrics
