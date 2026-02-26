#!/usr/bin/env python3

import argparse
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
except ModuleNotFoundError as exc:
    raise SystemExit(
        "matplotlib is required. Install it with: python3 -m pip install matplotlib"
    ) from exc
try:
    import pandas as pd
except ModuleNotFoundError as exc:
    raise SystemExit(
        "pandas is required. Install it with: python3 -m pip install pandas"
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot lap logs from lap_logs.csv")
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path(__file__).with_name("lap_logs.csv"),
        help="Path to lap_logs.csv (default: data/lap_logs.csv)",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional path to save figure (e.g. data/lap_plot.png)",
    )
    parser.add_argument(
        "--lap",
        type=int,
        default=None,
        help="Plot only one lap_id; by default plots all laps",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.csv.exists():
        raise FileNotFoundError(f"CSV file not found: {args.csv}")

    df = pd.read_csv(args.csv)
    required_cols = {"lap_id", "t", "cmd_speed", "cmd_thrust", "real_speed", "real_thrust"}
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"CSV is missing required columns: {sorted(missing)}")

    df = df.copy()
    df["lap_id"] = df["lap_id"].astype(int)
    if args.lap is not None:
        df = df[df["lap_id"] == args.lap]
        if df.empty:
            raise ValueError(f"No rows found for lap_id={args.lap}")

    if df.empty:
        raise ValueError("No valid rows found in CSV.")

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    speed_ax, thrust_ax = axes

    colors = {
        "cmd_speed": "#1f77b4",   # blue
        "real_speed": "#ff7f0e",  # orange
        "cmd_thrust": "#2ca02c",  # green
        "real_thrust": "#d62728", # red
    }
    line_width = 0.5  # about 1/3 of matplotlib default (~1.5)
    lap_ids = sorted(df["lap_id"].unique())
    if len(lap_ids) == 1:
        alpha_by_lap = {lap_ids[0]: 1.0}
    else:
        min_alpha = 0.08
        max_alpha = 1.0
        alpha_by_lap = {}
        for idx, lap_id in enumerate(lap_ids):
            alpha = min_alpha + (max_alpha - min_alpha) * idx / (len(lap_ids) - 1)
            alpha_by_lap[lap_id] = max(0.0, min(1.0, alpha))

    first_lap = lap_ids[0]
    for lap_id in lap_ids:
        g = df[df["lap_id"] == lap_id].sort_values("t")
        alpha = alpha_by_lap[lap_id]
        speed_ax.plot(
            g["t"],
            g["cmd_speed"],
            linestyle="-",
            color=colors["cmd_speed"],
            alpha=alpha,
            linewidth=line_width,
            label="cmd_speed" if lap_id == first_lap else None,
        )
        speed_ax.scatter(
            g["t"].iloc[-1],
            g["cmd_speed"].iloc[-1],
            color=colors["cmd_speed"],
            alpha=alpha,
            s=16,
            zorder=4,
        )
        speed_ax.plot(
            g["t"],
            g["real_speed"],
            linestyle="-",
            color=colors["real_speed"],
            alpha=alpha,
            linewidth=line_width,
            label="real_speed" if lap_id == first_lap else None,
        )
        speed_ax.scatter(
            g["t"].iloc[-1],
            g["real_speed"].iloc[-1],
            color=colors["real_speed"],
            alpha=alpha,
            s=16,
            zorder=4,
        )
        thrust_ax.plot(
            g["t"],
            g["cmd_thrust"],
            linestyle="-",
            color=colors["cmd_thrust"],
            alpha=alpha,
            linewidth=line_width,
            label="cmd_thrust" if lap_id == first_lap else None,
        )
        thrust_ax.scatter(
            g["t"].iloc[-1],
            g["cmd_thrust"].iloc[-1],
            color=colors["cmd_thrust"],
            alpha=alpha,
            s=16,
            zorder=4,
        )
        thrust_ax.plot(
            g["t"],
            g["real_thrust"],
            linestyle="-",
            color=colors["real_thrust"],
            alpha=alpha,
            linewidth=line_width,
            label="real_thrust" if lap_id == first_lap else None,
        )
        thrust_ax.scatter(
            g["t"].iloc[-1],
            g["real_thrust"].iloc[-1],
            color=colors["real_thrust"],
            alpha=alpha,
            s=16,
            zorder=4,
        )

    speed_ax.set_title("Speed: command vs real")
    speed_ax.set_ylabel("Speed")
    speed_ax.grid(True, alpha=0.3)

    thrust_ax.set_title("Thrust: command vs real")
    thrust_ax.set_xlabel("Time (s)")
    thrust_ax.set_ylabel("Thrust")
    thrust_ax.grid(True, alpha=0.3)

    speed_legend_handles = [
        Line2D([0], [0], color=colors["cmd_speed"], linestyle="-", linewidth=2.0, alpha=1.0, label="cmd_speed"),
        Line2D([0], [0], color=colors["real_speed"], linestyle="-", linewidth=2.0, alpha=1.0, label="real_speed"),
    ]
    thrust_legend_handles = [
        Line2D([0], [0], color=colors["cmd_thrust"], linestyle="-", linewidth=2.0, alpha=1.0, label="cmd_thrust"),
        Line2D([0], [0], color=colors["real_thrust"], linestyle="-", linewidth=2.0, alpha=1.0, label="real_thrust"),
    ]
    speed_ax.legend(handles=speed_legend_handles, loc="best")
    thrust_ax.legend(handles=thrust_legend_handles, loc="best")

    fig.tight_layout()

    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=200)
        print(f"Saved figure to: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
