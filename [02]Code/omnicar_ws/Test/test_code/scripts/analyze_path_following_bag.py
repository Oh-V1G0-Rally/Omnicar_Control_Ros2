from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import pandas as pd

from common import (
    SUMMARY_HEADERS,
    dataframe_to_csv,
    dump_json,
    dump_yaml,
    ensure_dir,
    load_path_reference,
    load_yaml,
    maybe_relative,
    validate_test_id,
    write_csv,
)
from compute_path_metrics import compute_metrics
from extract_debug_to_csv import extract_debug_dataframe
from generate_metrics_figure import generate_metrics_figure
from generate_tracking_figure import generate_tracking_figure
from generate_xy_figure import generate_xy_figure
from update_global_comparison_csv import update_global_comparison_csv


CONTROLLER_CONFIG_KEYS = [
    "controller_mode",
    "heading_mode",
    "desired_speed",
    "k_gamma",
    "min_gamma_dot",
    "max_gamma_dot",
    "linear_k_along",
    "linear_k_lateral",
    "linear_projection_lookahead",
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
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze a path following bag and generate plots and metrics.")
    parser.add_argument("--bag", required=True, help="Path to the bag directory")
    parser.add_argument("--test-id", required=True, help="Test ID using PF_YYYY-MM-DD_PATH_FRAME_CONTROLLER_VERSION_RUN")
    parser.add_argument("--path-name", required=True, help="Logical path name, for example RECT")
    parser.add_argument("--frame", required=True, choices=["MAP", "ODOM"], help="Reference frame of the test")
    parser.add_argument("--controller-name", required=True, help="Controller name, for example PID")
    parser.add_argument("--controller-version", required=True, help="Controller version, for example V01")
    parser.add_argument("--path-file", required=True, help="CSV path used as reference path")
    parser.add_argument("--config", required=True, help="Analysis configuration YAML")
    parser.add_argument("--output-root", required=True, help="Root folder for analyzed results")
    parser.add_argument("--map-config", help="Optional map YAML overriding the one in analysis config")
    parser.add_argument("--notes", default="", help="Optional test notes")
    parser.add_argument("--debug-topic", help="Optional debug topic override")
    parser.add_argument("--force", action="store_true", help="Allow overwriting an existing test result folder")
    parser.add_argument("--no-global-update", action="store_true", help="Skip the workspace global summary CSV update")
    return parser.parse_args()


def resolve_config_path(raw_path: str, config_path: Path) -> Path:
    path = Path(raw_path).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (config_path.parent / path).resolve()


def load_ros_parameters(path: Path) -> dict:
    payload = load_yaml(path)
    if "ros__parameters" in payload:
        return payload["ros__parameters"] or {}
    for node_cfg in payload.values():
        if isinstance(node_cfg, dict) and "ros__parameters" in node_cfg:
            return node_cfg["ros__parameters"] or {}
    return {}


def build_controller_config(config: dict, config_path: Path) -> tuple[dict, str]:
    controller_cfg = config["controller_defaults"].copy()
    controller_config_file = ""

    if config.get("controller_config"):
        controller_config_path = resolve_config_path(config["controller_config"], config_path)
        controller_config_file = str(controller_config_path)
        runtime_params = load_ros_parameters(controller_config_path)
        for key in CONTROLLER_CONFIG_KEYS:
            if key in runtime_params:
                controller_cfg[key] = runtime_params[key]

    return controller_cfg, controller_config_file


def build_summary_row(test_info: dict, metrics: dict, result_dir: Path) -> dict:
    row = {
        "test_id": test_info["test"]["test_id"],
        "date": test_info["test"]["date"],
        "run_index": test_info["test"]["run_index"],
        "bag_name": test_info["input"]["bag_name"],
        "path_name": test_info["scenario"]["path_name"],
        "frame": test_info["scenario"]["frame"],
        "map_name": test_info["scenario"]["map_name"],
        "controller_name": test_info["controller"]["controller_name"],
        "controller_version": test_info["controller"]["controller_version"],
        "controller_mode": test_info["controller"].get("controller_mode", ""),
        "heading_mode": test_info["controller"]["heading_mode"],
        "desired_speed": test_info["controller"]["desired_speed"],
        "k_gamma": test_info["controller"]["k_gamma"],
        "min_gamma_dot": test_info["controller"]["min_gamma_dot"],
        "max_gamma_dot": test_info["controller"]["max_gamma_dot"],
        "linear_k_along": test_info["controller"].get("linear_k_along", ""),
        "linear_k_lateral": test_info["controller"].get("linear_k_lateral", ""),
        "linear_projection_lookahead": test_info["controller"].get(
            "linear_projection_lookahead", ""),
        "control_rate_hz": test_info["controller"]["control_rate_hz"],
        "debug_publish_rate_hz": test_info["controller"]["debug_publish_rate_hz"],
        "entry_tolerance_xy": test_info["controller"]["entry_tolerance_xy"],
        "path_is_closed": test_info["controller"]["path_is_closed"],
        "stop_at_end": test_info["controller"]["stop_at_end"],
        "kp_x": test_info["controller"]["kp_x"],
        "ki_x": test_info["controller"]["ki_x"],
        "kd_x": test_info["controller"]["kd_x"],
        "kp_y": test_info["controller"]["kp_y"],
        "ki_y": test_info["controller"]["ki_y"],
        "kd_y": test_info["controller"]["kd_y"],
        "kp_yaw": test_info["controller"]["kp_yaw"],
        "ki_yaw": test_info["controller"]["ki_yaw"],
        "kd_yaw": test_info["controller"]["kd_yaw"],
        "max_linear_x": test_info["controller"]["max_linear_x"],
        "max_linear_y": test_info["controller"]["max_linear_y"],
        "max_angular_z": test_info["controller"]["max_angular_z"],
        "max_integral_x": test_info["controller"]["max_integral_x"],
        "max_integral_y": test_info["controller"]["max_integral_y"],
        "max_integral_yaw": test_info["controller"]["max_integral_yaw"],
        "result_folder": str(result_dir),
        "analysis_timestamp": test_info["analysis"]["analysis_timestamp"],
        "notes": test_info["test"]["notes"],
    }
    row.update(metrics)
    return {header: row.get(header, "") for header in SUMMARY_HEADERS}


def write_local_metrics_csv(path: Path, summary_row: dict) -> None:
    rows = [{"parameter": header, "value": summary_row.get(header, "")} for header in SUMMARY_HEADERS]
    dataframe_to_csv(path, pd.DataFrame(rows), ["parameter", "value"])


def main() -> None:
    args = parse_args()
    parts = validate_test_id(args.test_id)

    bag_path = Path(args.bag).resolve()
    path_file = Path(args.path_file).resolve()
    config_path = Path(args.config).resolve()
    output_root = Path(args.output_root).resolve()

    config = load_yaml(config_path)
    map_path = Path(args.map_config).resolve() if args.map_config else Path(config["map_config"]).resolve()
    map_cfg = load_yaml(map_path)

    result_dir = output_root / args.test_id
    if result_dir.exists() and not args.force:
        raise FileExistsError(f"Result folder '{result_dir}' already exists. Use --force to overwrite generated files.")

    raw_dir = ensure_dir(result_dir / "raw")
    figures_dir = ensure_dir(result_dir / "figures")
    metrics_dir = ensure_dir(result_dir / "metrics")

    debug_topic = args.debug_topic or config["topics"]["debug_topic"]
    yaw_arrow_stride = int(config["plot"]["yaw_arrow_stride"])
    path_reference = load_path_reference(path_file)
    df = extract_debug_dataframe(bag_path, debug_topic)
    metrics = compute_metrics(df, path_reference)

    dataframe_to_csv(raw_dir / "path_follower_debug.csv", df)
    dataframe_to_csv(raw_dir / "path_reference.csv", path_reference)

    controller_cfg, controller_config_file = build_controller_config(config, config_path)
    controller_cfg["controller_name"] = args.controller_name
    controller_cfg["controller_version"] = args.controller_version
    controller_cfg["debug_topic"] = debug_topic

    analysis_timestamp = datetime.now().isoformat(timespec="seconds")
    test_info = {
        "test": {
            "test_id": args.test_id,
            "test_type": "path_following",
            "date": parts["date"],
            "run_index": parts["run"],
            "notes": args.notes,
        },
        "input": {
            "bag_name": bag_path.name,
            "bag_folder": str(bag_path),
            "bag_file": f"{bag_path.name}_0.mcap",
            "rosbag_metadata_file": "metadata.yaml",
            "debug_topic": debug_topic,
        },
        "scenario": {
            "path_name": args.path_name,
            "path_file": str(path_file),
            "frame": args.frame,
            "map_name": map_cfg["map"]["name"],
            "map_width_m": map_cfg["map"]["width_m"],
            "map_height_m": map_cfg["map"]["height_m"],
            "pillars": map_cfg["map"]["pillars"],
        },
        "controller": controller_cfg,
        "analysis": {
            "analysis_timestamp": analysis_timestamp,
            "analysis_code_version": "1.0",
            "analysis_config_file": str(config_path),
            "controller_config_file": controller_config_file,
            "yaw_arrow_stride": yaw_arrow_stride,
            "unwrap_yaw": True,
            "output_folder": str(result_dir),
        },
        "outputs": {
            "raw_debug_csv": "raw/path_follower_debug.csv",
            "raw_path_csv": "raw/path_reference.csv",
            "xy_figure": "figures/01_xy_map.png",
            "tracking_figure": "figures/02_target_tracking_and_errors.png",
            "metrics_figure": "figures/03_metrics_summary.png",
            "local_metrics_csv": "metrics/metrics.csv",
            "local_metrics_json": "metrics/metrics.json",
        },
        "summary_metrics": metrics,
    }

    title = args.test_id
    generate_xy_figure(df, path_reference, map_cfg, figures_dir / "01_xy_map.png", title, yaw_arrow_stride)
    generate_tracking_figure(df, figures_dir / "02_target_tracking_and_errors.png", title)
    generate_metrics_figure(metrics, test_info, figures_dir / "03_metrics_summary.png", title)

    summary_row = build_summary_row(test_info, metrics, result_dir)
    write_local_metrics_csv(metrics_dir / "metrics.csv", summary_row)
    dump_json(metrics_dir / "metrics.json", metrics)
    dump_yaml(result_dir / "test_info.yaml", test_info)

    if not args.no_global_update:
        update_global_comparison_csv(output_root / "path_following_tests_summary.csv", summary_row)
    elif not (output_root / "path_following_tests_summary.csv").exists():
        write_csv(output_root / "path_following_tests_summary.csv", [], SUMMARY_HEADERS)


if __name__ == "__main__":
    main()
