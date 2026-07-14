#!/usr/bin/env python3

import csv
import sys
from pathlib import Path


def read_rows(csv_path):
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        row["t"] = float(row["t"])
        row["x"] = float(row["x"])
        row["y"] = float(row["y"])
        row["z"] = float(row["z"])
    return rows


def main():
    if len(sys.argv) != 3:
        print("usage: plot_jog_linearity.py trajectory.csv output_dir")
        return 2

    csv_path = Path(sys.argv[1])
    output_dir = Path(sys.argv[2])
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = read_rows(csv_path)
    axes = ["X", "Y", "Z"]

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(8, 7))
    ax3d = fig.add_subplot(111, projection="3d")
    for axis in axes:
        pts = [row for row in rows if row["axis"] == axis]
        ax3d.plot(
            [row["x"] for row in pts],
            [row["y"] for row in pts],
            [row["z"] for row in pts],
            marker=".",
            label=f"BASE_{axis}",
        )
    ax3d.set_xlabel("X / m")
    ax3d.set_ylabel("Y / m")
    ax3d.set_zlabel("Z / m")
    ax3d.legend()
    ax3d.set_title("Flange trajectory during Cartesian jog")
    fig.tight_layout()
    fig.savefig(output_dir / "trajectory_3d.png", dpi=160)
    plt.close(fig)

    fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    labels = [("x", "X / m"), ("y", "Y / m"), ("z", "Z / m")]
    for axis in axes:
        pts = [row for row in rows if row["axis"] == axis and row["t"] >= 0.0]
        t = [row["t"] for row in pts]
        for subplot, (key, ylabel) in zip(axs, labels):
            subplot.plot(t, [row[key] for row in pts], marker=".", label=f"BASE_{axis}")
            subplot.set_ylabel(ylabel)
            subplot.grid(True, alpha=0.3)
    axs[-1].set_xlabel("time / s")
    axs[0].legend()
    fig.tight_layout()
    fig.savefig(output_dir / "xyz_vs_time.png", dpi=160)
    plt.close(fig)

    print(output_dir / "trajectory_3d.png")
    print(output_dir / "xyz_vs_time.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
