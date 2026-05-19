from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


@dataclass
class RunMetrics:
    run_id: str
    gamma_start_m: float
    gamma_end_m: float
    gamma_common_m: float
    duration_s: float
    samples_common: int
    rmse_ec_m: float
    mae_ec_m: float
    max_abs_ec_m: float
    rmse_el_m: float
    mae_el_m: float
    max_abs_el_m: float


def _rmse(values: pd.Series) -> float:
    arr = pd.to_numeric(values, errors="coerce").dropna().to_numpy(dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(arr ** 2)))


def _mae(values: pd.Series) -> float:
    arr = pd.to_numeric(values, errors="coerce").dropna().to_numpy(dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(np.mean(np.abs(arr)))


def _max_abs(values: pd.Series) -> float:
    arr = pd.to_numeric(values, errors="coerce").dropna().to_numpy(dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(np.max(np.abs(arr)))


def _sanitize(
    df: pd.DataFrame,
    gamma_col: str,
    ec_col: str,
    el_col: str,
    time_col: str,
) -> pd.DataFrame:
    out = df.copy()

    for col in [gamma_col, ec_col, el_col, time_col]:
        out[col] = pd.to_numeric(out[col], errors="coerce")

    out = out.dropna(subset=[gamma_col, ec_col, el_col, time_col])
    out = out.sort_values(by=time_col).reset_index(drop=True)

    return out


def _compute_metrics_for_run(
    run_df: pd.DataFrame,
    run_id: str,
    gamma_col: str,
    ec_col: str,
    el_col: str,
    time_col: str,
    gamma_start_common_m: float,
    gamma_end_common_m: float,
) -> RunMetrics:
    seg = run_df[
        (run_df[gamma_col] >= gamma_start_common_m)
        & (run_df[gamma_col] <= gamma_end_common_m)
    ].copy()

    if seg.empty:
        raise ValueError(
            f"Run '{run_id}' has no samples in "
            f"[{gamma_start_common_m:.6f}, {gamma_end_common_m:.6f}] m"
        )

    duration_s = (
        float(seg[time_col].iloc[-1] - seg[time_col].iloc[0])
        if len(seg) > 1
        else 0.0
    )

    return RunMetrics(
        run_id=run_id,
        gamma_start_m=float(gamma_start_common_m),
        gamma_end_m=float(gamma_end_common_m),
        gamma_common_m=float(gamma_end_common_m - gamma_start_common_m),
        duration_s=duration_s,
        samples_common=int(len(seg)),
        rmse_ec_m=_rmse(seg[ec_col]),
        mae_ec_m=_mae(seg[ec_col]),
        max_abs_ec_m=_max_abs(seg[ec_col]),
        rmse_el_m=_rmse(seg[el_col]),
        mae_el_m=_mae(seg[el_col]),
        max_abs_el_m=_max_abs(seg[el_col]),
    )


def _format_float(value: float, digits: int = 4) -> str:
    if value is None or (
        isinstance(value, float) and (math.isnan(value) or math.isinf(value))
    ):
        return "nan"
    return f"{value:.{digits}f}"


def _to_latex_row(metrics: RunMetrics) -> str:
    return (
        f"{metrics.run_id} & "
        f"{_format_float(metrics.gamma_common_m, 3)} & "
        f"{_format_float(metrics.duration_s, 2)} & "
        f"{_format_float(metrics.rmse_ec_m)} & "
        f"{_format_float(metrics.mae_ec_m)} & "
        f"{_format_float(metrics.max_abs_ec_m)} & "
        f"{_format_float(metrics.rmse_el_m)} & "
        f"{_format_float(metrics.mae_el_m)} \\\\"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Compute common-segment contouring/lag metrics for selected runs "
            "and print LaTeX-ready table rows."
        )
    )

    parser.add_argument("--csv", type=Path, required=True, help="Input CSV file")
    parser.add_argument(
        "--run-col",
        default="run_id",
        help="Column containing run identifiers",
    )
    parser.add_argument(
        "--gamma-col",
        default="gamma",
        help="Column containing path coordinate gamma [m]",
    )
    parser.add_argument(
        "--ec-col",
        default="error_contouring",
        help="Column containing contouring error e_c [m]",
    )
    parser.add_argument(
        "--el-col",
        default="error_lag",
        help="Column containing lag error e_l [m]",
    )
    parser.add_argument(
        "--time-col",
        default="time_from_start_s",
        help="Column containing time from start [s]",
    )
    parser.add_argument(
        "--runs",
        nargs="+",
        required=True,
        help="Run IDs to include, for example: LIN-04 LIN-05",
    )
    parser.add_argument(
        "--gamma-start",
        type=float,
        default=None,
        help=(
            "Optional manual common gamma start [m]. "
            "If omitted, max(gamma_min) among selected runs is used."
        ),
    )
    parser.add_argument(
        "--gamma-end",
        type=float,
        default=None,
        help=(
            "Optional manual common gamma end [m]. "
            "If omitted, min(gamma_max) among selected runs is used."
        ),
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=None,
        help="Optional output CSV for computed metrics",
    )

    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    required = [
        args.run_col,
        args.gamma_col,
        args.ec_col,
        args.el_col,
        args.time_col,
    ]

    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(
            f"Missing required columns: {missing}\n"
            f"Available columns are: {list(df.columns)}"
        )

    selected = df[df[args.run_col].isin(args.runs)].copy()
    if selected.empty:
        raise ValueError(
            "No rows found for selected runs. "
            "Check the names passed with --runs."
        )

    run_frames: dict[str, pd.DataFrame] = {}
    gamma_mins: dict[str, float] = {}
    gamma_maxs: dict[str, float] = {}

    for run_id in args.runs:
        run_df = selected[selected[args.run_col] == run_id].copy()

        if run_df.empty:
            raise ValueError(f"Run '{run_id}' not found in CSV")

        run_df = _sanitize(
            run_df,
            gamma_col=args.gamma_col,
            ec_col=args.ec_col,
            el_col=args.el_col,
            time_col=args.time_col,
        )

        if run_df.empty:
            raise ValueError(f"Run '{run_id}' has no valid numeric samples")

        run_frames[run_id] = run_df
        gamma_mins[run_id] = float(run_df[args.gamma_col].min())
        gamma_maxs[run_id] = float(run_df[args.gamma_col].max())

    gamma_start_common_m = (
        args.gamma_start
        if args.gamma_start is not None
        else max(gamma_mins.values())
    )

    gamma_end_common_m = (
        args.gamma_end
        if args.gamma_end is not None
        else min(gamma_maxs.values())
    )

    if gamma_end_common_m <= gamma_start_common_m:
        raise ValueError(
            "Invalid common segment: "
            f"gamma_start={gamma_start_common_m:.6f}, "
            f"gamma_end={gamma_end_common_m:.6f}. "
            "The selected runs do not have an overlapping gamma interval."
        )

    metrics_rows: list[RunMetrics] = []

    for run_id in args.runs:
        metrics_rows.append(
            _compute_metrics_for_run(
                run_df=run_frames[run_id],
                run_id=run_id,
                gamma_col=args.gamma_col,
                ec_col=args.ec_col,
                el_col=args.el_col,
                time_col=args.time_col,
                gamma_start_common_m=gamma_start_common_m,
                gamma_end_common_m=gamma_end_common_m,
            )
        )

    print("\n[Info] Gamma range by run:")
    for run_id in args.runs:
        print(
            f"  - {run_id}: "
            f"gamma_min = {gamma_mins[run_id]:.6f} m, "
            f"gamma_max = {gamma_maxs[run_id]:.6f} m"
        )

    print(
        "\n[Info] Common segment used: "
        f"[{gamma_start_common_m:.6f}, {gamma_end_common_m:.6f}] m"
    )
    print(
        "[Info] Common segment length: "
        f"{gamma_end_common_m - gamma_start_common_m:.6f} m\n"
    )

    print("[LaTeX rows]")
    for row in metrics_rows:
        print(_to_latex_row(row))

    if args.output_csv is not None:
        args.output_csv.parent.mkdir(parents=True, exist_ok=True)

        out_df = pd.DataFrame(
            [
                {
                    "run": m.run_id,
                    "gamma_start_common_m": m.gamma_start_m,
                    "gamma_end_common_m": m.gamma_end_m,
                    "common_segment_m": m.gamma_common_m,
                    "duration_s": m.duration_s,
                    "samples_common": m.samples_common,
                    "rmse_ec_m": m.rmse_ec_m,
                    "mae_ec_m": m.mae_ec_m,
                    "max_abs_ec_m": m.max_abs_ec_m,
                    "rmse_el_m": m.rmse_el_m,
                    "mae_el_m": m.mae_el_m,
                    "max_abs_el_m": m.max_abs_el_m,
                }
                for m in metrics_rows
            ]
        )

        out_df.to_csv(args.output_csv, index=False)
        print(f"\n[Info] Metrics CSV written to: {args.output_csv}")


if __name__ == "__main__":
    main()
