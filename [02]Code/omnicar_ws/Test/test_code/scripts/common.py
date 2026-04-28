from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml


TEST_ID_PATTERN = re.compile(
    r"^PF_(?P<date>\d{4}-\d{2}-\d{2})_(?P<path>[A-Z0-9]+)_(?P<frame>MAP|ODOM)_"
    r"(?P<controller>[A-Z0-9]+)_(?P<version>V\d{2})_(?P<run>R\d{2})$"
)

SUMMARY_HEADERS = [
    "test_id",
    "date",
    "run_index",
    "bag_name",
    "path_name",
    "frame",
    "map_name",
    "controller_name",
    "controller_version",
    "controller_mode",
    "heading_mode",
    "desired_speed",
    "k_gamma",
    "min_gamma_dot",
    "max_gamma_dot",
    "linear_k_along",
    "linear_k_lateral",
    "linear_projection_lookahead",
    "linear_projection_search_behind",
    "linear_projection_search_ahead",
    "control_rate_hz",
    "debug_publish_rate_hz",
    "entry_tolerance_xy",
    "path_is_closed",
    "stop_at_end",
    "kp_x",
    "ki_x",
    "kd_x",
    "kp_y",
    "ki_y",
    "kd_y",
    "kp_yaw",
    "ki_yaw",
    "kd_yaw",
    "max_linear_x",
    "max_linear_y",
    "max_angular_z",
    "max_integral_x",
    "max_integral_y",
    "max_integral_yaw",
    "duration_s",
    "samples",
    "dt_mean_s",
    "path_length_real_m",
    "gamma_final_m",
    "gamma_progress_ratio",
    "rmse_x_m",
    "rmse_y_m",
    "rmse_tracking_error_norm_m",
    "mean_tracking_error_norm_m",
    "max_tracking_error_norm_m",
    "rmse_yaw_rad",
    "rmse_error_along_m",
    "mean_abs_error_along_m",
    "max_abs_error_along_m",
    "iae_error_along",
    "max_abs_error_yaw_rad",
    "sat_vx_ratio",
    "sat_vy_ratio",
    "sat_w_ratio",
    "result_folder",
    "analysis_timestamp",
    "notes",
]

RAW_HEADERS = [
    "sample_idx",
    "time_s",
    "time_from_start_s",
    "active",
    "state",
    "control_frame_id",
    "current_frame_id",
    "stop_reason",
    "current_x",
    "current_y",
    "current_yaw",
    "current_yaw_unwrapped",
    "target_x",
    "target_y",
    "target_yaw",
    "target_yaw_unwrapped",
    "tangent_x",
    "tangent_y",
    "gamma",
    "gamma_dot",
    "error_x_world",
    "error_y_world",
    "error_x_body",
    "error_y_body",
    "error_along",
    "error_yaw",
    "error_yaw_unwrapped",
    "integral_x",
    "integral_y",
    "integral_yaw",
    "derivative_x",
    "derivative_y",
    "derivative_yaw",
    "feedforward_vx",
    "feedforward_vy",
    "cmd_vx_raw",
    "cmd_vy_raw",
    "cmd_w_raw",
    "cmd_vx",
    "cmd_vy",
    "cmd_w",
    "saturated_vx",
    "saturated_vy",
    "saturated_w",
    "heading_arrow_x",
    "heading_arrow_y",
    "target_point_distance",
    "tracking_error_norm",
]


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def dump_yaml(path: Path, payload: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=False)


def dump_json(path: Path, payload: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=False)


def validate_test_id(test_id: str) -> dict[str, str]:
    match = TEST_ID_PATTERN.match(test_id)
    if not match:
        raise ValueError(
            "test_id must match PF_YYYY-MM-DD_PATH_FRAME_CONTROLLER_VERSION_RUN, "
            f"got '{test_id}'"
        )
    return match.groupdict()


def unwrap_series(series: pd.Series) -> pd.Series:
    if series.empty:
        return series
    values = pd.to_numeric(series, errors="coerce").astype(float).to_numpy()
    return pd.Series(np.unwrap(values), index=series.index)


def load_path_reference(path_file: Path) -> pd.DataFrame:
    df = pd.read_csv(path_file)
    lowered = {name.lower(): name for name in df.columns}
    if "x" not in lowered or "y" not in lowered:
        if df.shape[1] < 2:
            raise ValueError(f"path file '{path_file}' must contain x,y columns")
        df = df.iloc[:, :3].copy()
        df.columns = ["x", "y"] + (["yaw"] if df.shape[1] > 2 else [])
    else:
        rename_map = {lowered["x"]: "x", lowered["y"]: "y"}
        if "yaw" in lowered:
            rename_map[lowered["yaw"]] = "yaw"
        df = df.rename(columns=rename_map)
        keep = ["x", "y"] + (["yaw"] if "yaw" in df.columns else [])
        df = df[keep].copy()

    df["x"] = pd.to_numeric(df["x"], errors="coerce")
    df["y"] = pd.to_numeric(df["y"], errors="coerce")
    if "yaw" in df.columns:
        df["yaw"] = pd.to_numeric(df["yaw"], errors="coerce")
    else:
        df["yaw"] = np.nan

    dx = df["x"].diff().fillna(0.0)
    dy = df["y"].diff().fillna(0.0)
    ds = np.hypot(dx, dy)
    df["gamma"] = ds.cumsum()
    return df


def write_csv(path: Path, rows: list[dict[str, Any]], headers: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=headers)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in headers})


def dataframe_to_csv(path: Path, df: pd.DataFrame, headers: list[str] | None = None) -> None:
    if headers is not None:
        for column in headers:
            if column not in df.columns:
                df[column] = np.nan
        df = df[headers]
    df.to_csv(path, index=False)


def rmse(series: pd.Series) -> float:
    values = pd.to_numeric(series, errors="coerce").dropna().to_numpy()
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(values ** 2)))


def mae(series: pd.Series) -> float:
    values = pd.to_numeric(series, errors="coerce").dropna().to_numpy()
    if values.size == 0:
        return float("nan")
    return float(np.mean(np.abs(values)))


def max_abs(series: pd.Series) -> float:
    values = pd.to_numeric(series, errors="coerce").dropna().to_numpy()
    if values.size == 0:
        return float("nan")
    return float(np.max(np.abs(values)))


def float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return None
    return float(value)


def as_bool_ratio(series: pd.Series) -> float:
    if series.empty:
        return float("nan")
    normalized = series.astype(bool).astype(float)
    return float(normalized.mean())


def maybe_relative(path: Path, root: Path) -> str:
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)
