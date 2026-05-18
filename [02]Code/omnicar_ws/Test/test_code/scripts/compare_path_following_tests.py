from __future__ import annotations

import argparse
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from common import (
    dataframe_to_csv,
    dump_json,
    dump_yaml,
    ensure_dir,
    load_path_reference,
    load_yaml,
    validate_test_id,
)
from compute_path_metrics import compute_metrics
from extract_debug_to_csv import extract_debug_dataframe


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "analysis_config.yaml"
DEFAULT_RESULTS_ROOT = Path(__file__).resolve().parents[2] / "test_results"
REQUIRED_METRIC_KEYS = {
    "duration_s",
    "gamma_progress_ratio",
    "rmse_tracking_error_norm_m",
    "mean_tracking_error_norm_m",
    "max_tracking_error_norm_m",
    "rmse_error_along_m",
    "rmse_yaw_rad",
    "sat_vx_ratio",
    "sat_vy_ratio",
    "sat_w_ratio",
}
BAR_METRICS = [
    ("rmse_contouring_m", "RMSE contour", "m"),
    ("mae_contouring_m", "MAE contour", "m"),
    ("max_abs_contouring_m", "Max contour", "m"),
    ("rmse_lag_m", "RMSE lag", "m"),
    ("mae_lag_m", "MAE lag", "m"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare multiple path-following tests and draw all robot trajectories "
            "on the same map_xp figure."
        )
    )
    parser.add_argument(
        "--tests",
        nargs="+",
        required=True,
        help=(
            "Test folders or test IDs. Each item can be an analyzed result folder, "
            "a bag folder, or a test ID resolved under --results-root/--bags-root."
        ),
    )
    parser.add_argument("--results-root", default=str(DEFAULT_RESULTS_ROOT), help="Root folder with analyzed tests")
    parser.add_argument("--bags-root", help="Optional root folder with raw bag test directories")
    parser.add_argument("--path-file", help="Reference path CSV. Required when comparing raw bag folders")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Analysis configuration YAML")
    parser.add_argument("--map-config", help="Optional map YAML overriding the one in analysis config")
    parser.add_argument("--debug-topic", help="Optional debug topic override for raw bag folders")
    parser.add_argument("--output-dir", help="Output folder for comparison artifacts")
    parser.add_argument("--title", default="Path-following comparison", help="Figure title")
    parser.add_argument(
        "--label-field",
        default="controller_name",
        choices=["test_id", "controller_name", "controller_version", "desired_speed"],
        help="Primary metadata field used in the plot legend",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Allow writing into an existing non-empty output folder",
    )
    return parser.parse_args()


def resolve_existing_path(raw: str, results_root: Path, bags_root: Path | None) -> Path:
    candidate = Path(raw).expanduser()
    if candidate.exists():
        return candidate.resolve()

    result_candidate = results_root / raw
    if result_candidate.exists():
        return result_candidate.resolve()

    if bags_root is not None:
        bag_candidate = bags_root / raw
        if bag_candidate.exists():
            return bag_candidate.resolve()

    raise FileNotFoundError(
        f"Cannot resolve '{raw}' as a path, a result folder under '{results_root}', "
        "or a bag folder under --bags-root."
    )


def is_analyzed_result_folder(path: Path) -> bool:
    return (path / "raw" / "path_follower_debug.csv").exists()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    return payload if isinstance(payload, dict) else {}


def read_local_metrics(path: Path) -> dict[str, Any]:
    json_path = path / "metrics" / "metrics.json"
    if json_path.exists():
        return load_json(json_path)

    csv_path = path / "metrics" / "metrics.csv"
    if not csv_path.exists():
        return {}

    df = pd.read_csv(csv_path)
    if {"parameter", "value"}.issubset(df.columns):
        return dict(zip(df["parameter"], df["value"]))
    return {}


def read_test_info(path: Path) -> dict[str, Any]:
    info_path = path / "test_info.yaml"
    if info_path.exists():
        return load_yaml(info_path)
    return {}


def compute_axis_error_metrics(df: pd.DataFrame) -> dict[str, float]:
    error_x = (
        pd.to_numeric(df["current_x"], errors="coerce")
        - pd.to_numeric(df["target_x"], errors="coerce")
    )
    error_y = (
        pd.to_numeric(df["current_y"], errors="coerce")
        - pd.to_numeric(df["target_y"], errors="coerce")
    )
    return {
        "max_abs_error_x_m": float(error_x.abs().max()),
        "max_abs_error_y_m": float(error_y.abs().max()),
    }


def complete_metrics(df: pd.DataFrame, path_reference: pd.DataFrame | None, metrics: dict[str, Any]) -> dict[str, Any]:
    if REQUIRED_METRIC_KEYS.issubset(metrics.keys()) and {"max_abs_error_x_m", "max_abs_error_y_m"}.issubset(metrics.keys()):
        return metrics
    recomputed = compute_metrics(df, path_reference)
    recomputed.update(compute_axis_error_metrics(df))
    merged = metrics.copy()
    for key, value in recomputed.items():
        if key not in merged or merged[key] == "":
            merged[key] = value
    return merged


def metadata_from_info(test_id: str, info: dict[str, Any], metrics: dict[str, Any]) -> dict[str, Any]:
    controller = info.get("controller", {})
    scenario = info.get("scenario", {})
    test = info.get("test", {})
    parsed: dict[str, str] = {}
    try:
        parsed = validate_test_id(test_id)
    except ValueError:
        parsed = {}

    return {
        "test_id": test.get("test_id", test_id),
        "date": test.get("date", parsed.get("date", "")),
        "path_name": scenario.get("path_name", parsed.get("path", "")),
        "frame": scenario.get("frame", parsed.get("frame", "")),
        "run_index": test.get("run_index", parsed.get("run", "")),
        "controller_name": controller.get("controller_name", parsed.get("controller", "")),
        "controller_version": controller.get("controller_version", parsed.get("version", "")),
        "desired_speed": controller.get("desired_speed", ""),
        "max_linear_x": controller.get("max_linear_x", ""),
        "max_linear_y": controller.get("max_linear_y", ""),
        "max_angular_z": controller.get("max_angular_z", ""),
        "duration_s": metrics.get("duration_s", ""),
        "path_length_real_m": metrics.get("path_length_real_m", ""),
        "gamma_progress_ratio": metrics.get("gamma_progress_ratio", ""),
        "rmse_x_m": metrics.get("rmse_x_m", ""),
        "rmse_y_m": metrics.get("rmse_y_m", ""),
        "max_abs_error_x_m": metrics.get("max_abs_error_x_m", ""),
        "max_abs_error_y_m": metrics.get("max_abs_error_y_m", ""),
        "rmse_tracking_error_norm_m": metrics.get("rmse_tracking_error_norm_m", ""),
        "mean_tracking_error_norm_m": metrics.get("mean_tracking_error_norm_m", ""),
        "max_tracking_error_norm_m": metrics.get("max_tracking_error_norm_m", ""),
        "rmse_error_along_m": metrics.get("rmse_error_along_m", ""),
        "mean_abs_error_along_m": metrics.get("mean_abs_error_along_m", ""),
        "max_abs_error_along_m": metrics.get("max_abs_error_along_m", ""),
        "rmse_yaw_rad": metrics.get("rmse_yaw_rad", ""),
        "max_abs_error_yaw_rad": metrics.get("max_abs_error_yaw_rad", ""),
        "sat_vx_ratio": metrics.get("sat_vx_ratio", ""),
        "sat_vy_ratio": metrics.get("sat_vy_ratio", ""),
        "sat_w_ratio": metrics.get("sat_w_ratio", ""),
    }


def load_test_entry(
    path: Path,
    debug_topic: str,
    fallback_path_reference: pd.DataFrame | None,
) -> dict[str, Any]:
    if is_analyzed_result_folder(path):
        df = pd.read_csv(path / "raw" / "path_follower_debug.csv")
        path_reference_path = path / "raw" / "path_reference.csv"
        path_reference = pd.read_csv(path_reference_path) if path_reference_path.exists() else fallback_path_reference
        info = read_test_info(path)
        metrics = read_local_metrics(path)
        metrics = complete_metrics(df, path_reference, metrics)
        test_id = info.get("test", {}).get("test_id", path.name)
    else:
        if fallback_path_reference is None:
            raise ValueError("--path-file is required when at least one selected folder is a raw bag folder")
        df = extract_debug_dataframe(path, debug_topic)
        path_reference = fallback_path_reference
        metrics = compute_metrics(df, path_reference)
        info = {}
        test_id = path.name

    return {
        "source_folder": str(path),
        "test_id": test_id,
        "df": df,
        "path_reference": path_reference if path_reference is not None else pd.DataFrame(),
        "metrics": metrics,
        "info": info,
        "metadata": metadata_from_info(test_id, info, metrics),
    }


def numeric(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return result if math.isfinite(result) else float("nan")


def max_finite(values: list[float]) -> float:
    finite_values = [value for value in values if not math.isnan(value)]
    return max(finite_values) if finite_values else float("nan")


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def truncate_path_reference(path_reference: pd.DataFrame, gamma_start_m: float, gamma_end_m: float) -> pd.DataFrame:
    if path_reference.empty:
        return path_reference.copy()

    reference = path_reference.sort_values("gamma").reset_index(drop=True)
    gamma_values = pd.to_numeric(reference["gamma"], errors="coerce").to_numpy(dtype=float)

    def interpolated_row(gamma_m: float) -> dict[str, float]:
        row = {
            "x": float(np.interp(gamma_m, gamma_values, reference["x"].to_numpy(dtype=float))),
            "y": float(np.interp(gamma_m, gamma_values, reference["y"].to_numpy(dtype=float))),
            "gamma": gamma_m,
        }
        if "yaw" in reference.columns:
            yaw_unwrapped = np.unwrap(reference["yaw"].to_numpy(dtype=float))
            row["yaw"] = float(np.interp(gamma_m, gamma_values, yaw_unwrapped))
        return row

    mask = (reference["gamma"] >= gamma_start_m) & (reference["gamma"] <= gamma_end_m)
    truncated = reference[mask].copy()
    rows = [interpolated_row(gamma_start_m)]
    if not truncated.empty:
        rows.extend(truncated.to_dict("records"))
    rows.append(interpolated_row(gamma_end_m))
    truncated = pd.DataFrame(rows)
    truncated = truncated.drop_duplicates(subset=["gamma"], keep="first")

    return truncated.reset_index(drop=True)


def reference_at_gamma(path_reference: pd.DataFrame, gamma: np.ndarray) -> dict[str, np.ndarray]:
    reference = path_reference.sort_values("gamma").reset_index(drop=True)
    gamma_ref = pd.to_numeric(reference["gamma"], errors="coerce").to_numpy(dtype=float)
    x_ref = pd.to_numeric(reference["x"], errors="coerce").to_numpy(dtype=float)
    y_ref = pd.to_numeric(reference["y"], errors="coerce").to_numpy(dtype=float)

    if "yaw" in reference.columns and not reference["yaw"].isna().all():
        yaw_ref = np.unwrap(pd.to_numeric(reference["yaw"], errors="coerce").to_numpy(dtype=float))
    else:
        dx = np.gradient(x_ref, gamma_ref, edge_order=1)
        dy = np.gradient(y_ref, gamma_ref, edge_order=1)
        yaw_ref = np.unwrap(np.arctan2(dy, dx))

    return {
        "x": np.interp(gamma, gamma_ref, x_ref),
        "y": np.interp(gamma, gamma_ref, y_ref),
        "yaw": np.interp(gamma, gamma_ref, yaw_ref),
    }


def truncate_df_to_common_gamma(df: pd.DataFrame, gamma_start_m: float, gamma_end_m: float) -> pd.DataFrame:
    gamma = pd.to_numeric(df["gamma"], errors="coerce")
    truncated = df[(gamma >= gamma_start_m) & (gamma <= gamma_end_m)].copy()
    if truncated.empty:
        truncated = df.iloc[[0]].copy()
    return truncated.reset_index(drop=True)


def compute_common_segment_metrics(df: pd.DataFrame, path_reference: pd.DataFrame) -> dict[str, float]:
    if df.empty:
        raise ValueError("Cannot compute common segment metrics from an empty dataframe")

    gamma = pd.to_numeric(df["gamma"], errors="coerce").to_numpy(dtype=float)
    current_x = pd.to_numeric(df["current_x"], errors="coerce").to_numpy(dtype=float)
    current_y = pd.to_numeric(df["current_y"], errors="coerce").to_numpy(dtype=float)
    current_yaw = pd.to_numeric(df["current_yaw"], errors="coerce").to_numpy(dtype=float)

    reference = reference_at_gamma(path_reference, gamma)
    error_x = current_x - reference["x"]
    error_y = current_y - reference["y"]

    tangent_x = np.cos(reference["yaw"])
    tangent_y = np.sin(reference["yaw"])
    lag_error = error_x * tangent_x + error_y * tangent_y
    contouring_error = -error_x * tangent_y + error_y * tangent_x
    yaw_error = wrap_angle(current_yaw - reference["yaw"])

    return {
        "duration_common_s": float(df["time_from_start_s"].iloc[-1] - df["time_from_start_s"].iloc[0]),
        "samples_common": int(len(df)),
        "gamma_min_common_m": float(np.min(gamma)),
        "gamma_max_common_m": float(np.max(gamma)),
        "rmse_contouring_m": float(np.sqrt(np.mean(contouring_error ** 2))),
        "mae_contouring_m": float(np.mean(np.abs(contouring_error))),
        "max_abs_contouring_m": float(np.max(np.abs(contouring_error))),
        "rmse_lag_m": float(np.sqrt(np.mean(lag_error ** 2))),
        "mae_lag_m": float(np.mean(np.abs(lag_error))),
        "max_abs_lag_m": float(np.max(np.abs(lag_error))),
        "rmse_yaw_ref_rad": float(np.sqrt(np.mean(yaw_error ** 2))),
        "max_abs_yaw_ref_rad": float(np.max(np.abs(yaw_error))),
    }


def apply_common_segment(entries: list[dict[str, Any]], path_reference: pd.DataFrame) -> tuple[float, pd.DataFrame]:
    gamma_min_m = max(float(pd.to_numeric(entry["df"]["gamma"], errors="coerce").min()) for entry in entries)
    gamma_max_m = min(float(pd.to_numeric(entry["df"]["gamma"], errors="coerce").max()) for entry in entries)
    if gamma_max_m <= gamma_min_m:
        raise ValueError(
            "Selected tests do not have an overlapping gamma interval; "
            f"got start={gamma_min_m:.4f} m and end={gamma_max_m:.4f} m."
        )
    common_reference = truncate_path_reference(path_reference, gamma_min_m, gamma_max_m)

    for entry in entries:
        df_common = truncate_df_to_common_gamma(entry["df"], gamma_min_m, gamma_max_m)
        entry["df_common"] = df_common
        entry["common_gamma_start_m"] = gamma_min_m
        entry["common_gamma_end_m"] = gamma_max_m
        entry["common_segment_length_m"] = gamma_max_m - gamma_min_m
        entry["common_metrics"] = compute_common_segment_metrics(df_common, common_reference)

    return gamma_max_m - gamma_min_m, common_reference


def build_comparison_rows(entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for entry in entries:
        row = {
            "test_id": entry["metadata"].get("test_id", entry["test_id"]),
            "date": entry["metadata"].get("date", ""),
            "path_name": entry["metadata"].get("path_name", ""),
            "frame": entry["metadata"].get("frame", ""),
            "run_index": entry["metadata"].get("run_index", ""),
            "controller_name": entry["metadata"].get("controller_name", ""),
            "controller_version": entry["metadata"].get("controller_version", ""),
            "desired_speed": entry["metadata"].get("desired_speed", ""),
            "common_gamma_start_m": entry["common_gamma_start_m"],
            "common_gamma_end_m": entry["common_gamma_end_m"],
            "common_segment_length_m": entry["common_segment_length_m"],
            **entry["common_metrics"],
            "source_folder": entry["source_folder"],
        }
        rows.append(row)
    return rows


def assign_colors(entries: list[dict[str, Any]]) -> dict[str, str]:
    color_cycle = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    return {entry["test_id"]: color_cycle[index % len(color_cycle)] for index, entry in enumerate(entries)}


def legend_label(entry: dict[str, Any], label_field: str) -> str:
    metadata = entry["metadata"]
    primary = str(metadata.get(label_field, "") or entry["test_id"])
    speed = metadata.get("desired_speed", "")
    version = metadata.get("controller_version", "")
    test_id = metadata.get("test_id", entry["test_id"])
    run_index = metadata.get("run_index", "")

    suffix_parts = [part for part in [version, f"v={speed}" if speed != "" else ""] if part]
    suffix = f" ({', '.join(suffix_parts)})" if suffix_parts else ""
    if label_field == "test_id":
        return f"{primary}{suffix}"
    run_suffix = f" {run_index}" if run_index else ""
    return f"{primary}{suffix}{run_suffix}"


def plot_map_comparison(
    entries: list[dict[str, Any]],
    path_reference: pd.DataFrame,
    map_cfg: dict[str, Any],
    output_path: Path,
    title: str,
    label_field: str,
    color_by_test: dict[str, str],
) -> None:
    fig, ax = plt.subplots(figsize=(9.0, 10.5))
    draw_map_axis(ax, entries, path_reference, map_cfg, title, label_field, color_by_test)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=min(3, max(1, len(labels))),
        borderaxespad=0.0,
        fontsize=8,
    )
    fig.subplots_adjust(bottom=0.20)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def draw_map_axis(
    ax: plt.Axes,
    entries: list[dict[str, Any]],
    path_reference: pd.DataFrame,
    map_cfg: dict[str, Any],
    title: str,
    label_field: str,
    color_by_test: dict[str, str],
) -> None:

    width = float(map_cfg["map"]["width_m"])
    height = float(map_cfg["map"]["height_m"])
    origin_x = -width / 2.0
    origin_y = -height / 2.0
    ax.add_patch(plt.Rectangle((origin_x, origin_y), width, height, fill=False, linewidth=2.0, color="black"))
    ax.text(
        0.0,
        origin_y - 0.08,
        f"{width * 100:.0f} cm x {height * 100:.0f} cm",
        ha="center",
        va="top",
        fontsize=9,
        color="black",
    )

    for pillar in map_cfg["map"].get("pillars", []):
        ax.scatter(pillar[0], pillar[1], s=80, marker="s", color="dimgray")

    if not path_reference.empty:
        ax.plot(
            path_reference["x"],
            path_reference["y"],
            color="black",
            linewidth=2.0,
            linestyle="--",
            alpha=0.7,
            label="Reference path",
        )

    for entry in entries:
        df = entry.get("df_common", entry["df"])
        color = color_by_test[entry["test_id"]]
        label = legend_label(entry, label_field)
        ax.plot(df["current_x"], df["current_y"], color=color, linewidth=1.8, label=label)
        ax.scatter(df["current_x"].iloc[0], df["current_y"].iloc[0], color=color, marker="o", s=35, alpha=0.85)
        ax.scatter(df["current_x"].iloc[-1], df["current_y"].iloc[-1], color=color, marker="x", s=55, alpha=0.95)

    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.0, 1.0)
    ax.grid(True, alpha=0.3)


def draw_metric_bars_axis(
    ax: plt.Axes,
    rows: list[dict[str, Any]],
    color_by_test: dict[str, str],
) -> None:
    metric_count = len(BAR_METRICS)
    test_count = len(rows)
    x_positions = np.arange(metric_count)
    total_width = 0.82
    bar_width = total_width / max(test_count, 1)
    first_offset = -total_width / 2.0 + bar_width / 2.0

    for test_index, row in enumerate(rows):
        values = []
        for column, _, _ in BAR_METRICS:
            value = numeric(row.get(column))
            values.append(0.0 if math.isnan(value) else value)
        color = color_by_test.get(row["test_id"], "tab:blue")
        ax.bar(
            x_positions + first_offset + test_index * bar_width,
            values,
            width=bar_width * 0.92,
            color=color,
            alpha=0.9,
        )

    labels = [f"{label}\n[{unit}]" for _, label, unit in BAR_METRICS]
    ax.set_xticks(x_positions)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Metric value")
    ax.grid(True, axis="y", alpha=0.3)


def plot_metric_bars(rows: list[dict[str, Any]], color_by_test: dict[str, str], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(11.5, 4.8))
    draw_metric_bars_axis(ax, rows, color_by_test)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def plot_combined_comparison(
    entries: list[dict[str, Any]],
    rows: list[dict[str, Any]],
    path_reference: pd.DataFrame,
    map_cfg: dict[str, Any],
    output_path: Path,
    title: str,
    label_field: str,
    color_by_test: dict[str, str],
) -> None:
    fig = plt.figure(figsize=(11.5, 10.5))
    grid = fig.add_gridspec(3, 1, height_ratios=[1.75, 0.28, 1.55], hspace=0.18)
    map_ax = fig.add_subplot(grid[0, 0])
    legend_ax = fig.add_subplot(grid[1, 0])
    bars_ax = fig.add_subplot(grid[2, 0])

    draw_map_axis(map_ax, entries, path_reference, map_cfg, title, label_field, color_by_test)
    handles, labels = map_ax.get_legend_handles_labels()
    legend_ax.axis("off")
    legend_ax.legend(
        handles,
        labels,
        loc="center",
        ncol=min(3, max(1, len(labels))),
        frameon=True,
        fontsize=8,
    )
    draw_metric_bars_axis(bars_ax, rows, color_by_test)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def write_recommendation(rows: list[dict[str, Any]], output_path: Path) -> None:
    lines = [
        "# Common-segment path-following comparison",
        "",
        "All metrics are computed only on the common trajectory segment reached by every selected test.",
        "The reference is the same geometric path for every controller, interpolated at the recorded path coordinate.",
        "",
        "## Metrics",
        "",
        "| test_id | controller | desired_speed | common_segment_m | rmse_contour_m | mae_contour_m | max_contour_m | rmse_lag_m | mae_lag_m |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| "
            f"{row.get('test_id', '')} | {row.get('controller_name', '')} | "
            f"{row.get('desired_speed', '')} | "
            f"{numeric(row.get('common_segment_length_m')):.4g} | "
            f"{numeric(row.get('rmse_contouring_m')):.4g} | "
            f"{numeric(row.get('mae_contouring_m')):.4g} | "
            f"{numeric(row.get('max_abs_contouring_m')):.4g} | "
            f"{numeric(row.get('rmse_lag_m')):.4g} | "
            f"{numeric(row.get('mae_lag_m')):.4g} |"
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).expanduser().resolve()
    results_root = Path(args.results_root).expanduser().resolve()
    bags_root = Path(args.bags_root).expanduser().resolve() if args.bags_root else None

    config = load_yaml(config_path)
    map_path = Path(args.map_config).expanduser().resolve() if args.map_config else Path(config["map_config"]).resolve()
    map_cfg = load_yaml(map_path)
    debug_topic = args.debug_topic or config["topics"]["debug_topic"]

    fallback_path_reference = load_path_reference(Path(args.path_file).expanduser().resolve()) if args.path_file else None
    selected_paths = [resolve_existing_path(raw, results_root, bags_root) for raw in args.tests]
    entries = [load_test_entry(path, debug_topic, fallback_path_reference) for path in selected_paths]

    path_reference = fallback_path_reference
    if path_reference is None:
        for entry in entries:
            if not entry["path_reference"].empty:
                path_reference = entry["path_reference"]
                break
    if path_reference is None:
        path_reference = pd.DataFrame()

    common_segment_length_m, path_reference_common = apply_common_segment(entries, path_reference)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else results_root / "comparisons" / f"path_following_comparison_{timestamp}"
    )
    if output_dir.exists() and any(output_dir.iterdir()) and not args.force:
        raise FileExistsError(f"Output folder '{output_dir}' is not empty. Use --force to reuse it.")
    ensure_dir(output_dir)
    figures_dir = ensure_dir(output_dir / "figures")
    tables_dir = ensure_dir(output_dir / "tables")

    rows = build_comparison_rows(entries)
    color_by_test = assign_colors(entries)
    dataframe_to_csv(tables_dir / "comparison_metrics.csv", pd.DataFrame(rows))
    dump_json(tables_dir / "comparison_metrics.json", {"tests": rows})
    dump_yaml(
        output_dir / "comparison_info.yaml",
        {
            "analysis_timestamp": datetime.now().isoformat(timespec="seconds"),
            "config_file": str(config_path),
            "map_config_file": str(map_path),
            "debug_topic": debug_topic,
            "comparison_policy": "common_segment_only",
            "common_segment_length_m": common_segment_length_m,
            "common_gamma_start_m": entries[0]["common_gamma_start_m"],
            "common_gamma_end_m": entries[0]["common_gamma_end_m"],
            "selected_tests": [entry["test_id"] for entry in entries],
            "source_folders": [entry["source_folder"] for entry in entries],
            "outputs": {
                "map_xp": "figures/map_xp.png",
                "metrics_bars": "figures/comparison_metrics.png",
                "comparison_csv": "tables/comparison_metrics.csv",
                "recommendation": "recommendation.md",
            },
        },
    )

    plot_map_comparison(
        entries,
        path_reference_common,
        map_cfg,
        figures_dir / "map_xp.png",
        args.title,
        args.label_field,
        color_by_test,
    )
    plot_combined_comparison(
        entries,
        rows,
        path_reference_common,
        map_cfg,
        figures_dir / "comparison_metrics.png",
        args.title,
        args.label_field,
        color_by_test,
    )
    write_recommendation(rows, output_dir / "recommendation.md")

    print(f"Wrote comparison to: {output_dir}")
    print(f"Common segment length: {common_segment_length_m:.4f} m")


if __name__ == "__main__":
    main()
