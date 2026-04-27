from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from common import RAW_HEADERS, unwrap_series


MESSAGE_FIELDS = [
    "active",
    "state",
    "control_frame_id",
    "current_frame_id",
    "stop_reason",
    "current_x",
    "current_y",
    "current_yaw",
    "target_x",
    "target_y",
    "target_yaw",
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
]


def _load_rosbag_modules() -> tuple[Any, Any, Any, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 bag dependencies are not available. Source the ROS2 workspace before running "
            "this script so rosbag2_py, rclpy and rosidl_runtime_py are importable."
        ) from exc
    return rosbag2_py, deserialize_message, get_message, object()


def extract_debug_dataframe(bag_path: Path, debug_topic: str) -> pd.DataFrame:
    rosbag2_py, deserialize_message, get_message, _ = _load_rosbag_modules()

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if debug_topic not in topics:
        raise ValueError(f"Topic '{debug_topic}' not found in bag '{bag_path}'")

    msg_type = get_message(topics[debug_topic])
    rows: list[dict[str, Any]] = []
    sample_idx = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != debug_topic:
            continue

        msg = deserialize_message(data, msg_type)
        row = {
            "sample_idx": sample_idx,
            "time_s": float(timestamp) / 1e9,
        }

        for field in MESSAGE_FIELDS:
            row[field] = getattr(msg, field, None)

        rows.append(row)
        sample_idx += 1

    if not rows:
        raise ValueError(f"No messages found on topic '{debug_topic}' in '{bag_path}'")

    df = pd.DataFrame(rows)
    df["time_from_start_s"] = df["time_s"] - float(df["time_s"].iloc[0])
    df["current_yaw_unwrapped"] = unwrap_series(df["current_yaw"])
    df["target_yaw_unwrapped"] = unwrap_series(df["target_yaw"])
    df["error_yaw_unwrapped"] = df["target_yaw_unwrapped"] - df["current_yaw_unwrapped"]
    df["heading_arrow_x"] = np.cos(pd.to_numeric(df["current_yaw"], errors="coerce"))
    df["heading_arrow_y"] = np.sin(pd.to_numeric(df["current_yaw"], errors="coerce"))
    df["target_point_distance"] = np.hypot(
        pd.to_numeric(df["target_x"], errors="coerce") - pd.to_numeric(df["current_x"], errors="coerce"),
        pd.to_numeric(df["target_y"], errors="coerce") - pd.to_numeric(df["current_y"], errors="coerce"),
    )
    df["tracking_error_norm"] = np.hypot(
        pd.to_numeric(df["error_x_world"], errors="coerce"),
        pd.to_numeric(df["error_y_world"], errors="coerce"),
    )

    for column in RAW_HEADERS:
        if column not in df.columns:
            df[column] = np.nan

    return df[RAW_HEADERS]
