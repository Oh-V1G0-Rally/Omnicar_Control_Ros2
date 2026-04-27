from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def generate_xy_figure(
    df: pd.DataFrame,
    path_reference: pd.DataFrame,
    map_cfg: dict,
    output_path: Path,
    title: str,
    yaw_arrow_stride: int,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 10))

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
            color="tab:purple",
            linewidth=1.8,
            linestyle="--",
            label="Target path",
        )

    ax.plot(df["current_x"], df["current_y"], color="tab:red", linewidth=1.8, label="Robot path")
    ax.scatter(df["current_x"].iloc[0], df["current_y"].iloc[0], color="green", s=80, label="Start")
    ax.scatter(df["current_x"].iloc[-1], df["current_y"].iloc[-1], color="black", s=80, label="End")

    stride = max(int(yaw_arrow_stride), 1)
    sampled = df.iloc[::stride]
    ax.quiver(
        sampled["current_x"],
        sampled["current_y"],
        sampled["heading_arrow_x"],
        sampled["heading_arrow_y"],
        angles="xy",
        scale_units="xy",
        scale=10.0,
        width=0.003,
        color="tab:blue",
        alpha=0.8,
        label="Heading",
    )

    target_stride = max(stride, 1)
    sampled_target = df.iloc[::target_stride]
    ax.scatter(
        sampled_target["target_x"],
        sampled_target["target_y"],
        s=12,
        color="tab:orange",
        alpha=0.7,
        label="Sampled target points",
    )

    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.0, 1.0)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), borderaxespad=0.0)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
