#!/usr/bin/env python3
"""
SVD 零空间点动 — 末端漂移累积可视化

用法：
  python3 test/plot_svd_drift.py build/svd_drift.csv [output_dir]
"""

import csv
import math
import sys
import os
from pathlib import Path


def load_csv(path: str) -> list[dict]:
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "build/svd_drift.csv"
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("build/svd_drift_plots")
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = load_csv(csv_path)
    print(f"加载 {len(rows)} 个采样点")

    t  = [r["t"]  for r in rows]
    px = [r["px"] * 1000 for r in rows]  # mm
    py = [r["py"] * 1000 for r in rows]
    pz = [r["pz"] * 1000 for r in rows]

    px0, py0, pz0 = px[0], py[0], pz[0]
    dx = [x - px0 for x in px]  # mm 偏差
    dy = [y - py0 for y in py]
    dz = [z - pz0 for z in pz]
    drift = [math.sqrt(dx[i]**2 + dy[i]**2 + dz[i]**2) for i in range(len(t))]

    # 旋转 Frobenius 偏差
    def frob(i: int) -> float:
        r0, r = rows[0], rows[i]
        sq = lambda v: v*v
        return math.sqrt(
            sq(r["m00"]-r0["m00"])+sq(r["m01"]-r0["m01"])+sq(r["m02"]-r0["m02"])+
            sq(r["m10"]-r0["m10"])+sq(r["m11"]-r0["m11"])+sq(r["m12"]-r0["m12"])+
            sq(r["m20"]-r0["m20"])+sq(r["m21"]-r0["m21"])+sq(r["m22"]-r0["m22"])
        )
    rfrob = [frob(i) for i in range(len(t))]

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 未安装，仅输出统计信息")

        print(f"\n漂移统计 ({t[-1]:.0f}s):")
        print(f"  末端位置漂移: {drift[-1]*1000:.3f} μm")
        print(f"    X: {dx[-1]*1000:.3f} μm")
        print(f"    Y: {dy[-1]*1000:.3f} μm")
        print(f"    Z: {dz[-1]*1000:.3f} μm")
        print(f"  旋转漂移:     {rfrob[-1]:.6f} (Frobenius)")
        print(f"  漂移速率:     {drift[-1]/t[-1]*1e6:.2f} nm/s")
        return

    # ── 图1: 位置偏差 vs 时间 ──
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("SVD 零空间点动 — 末端位姿漂移 (离线规划层)", fontsize=14)

    # (a) 各轴偏差
    ax = axes[0][0]
    ax.plot(t, [d * 1000 for d in dx], label="ΔX", linewidth=0.8)
    ax.plot(t, [d * 1000 for d in dy], label="ΔY", linewidth=0.8)
    ax.plot(t, [d * 1000 for d in dz], label="ΔZ", linewidth=0.8)
    ax.axhline(y=0, color="gray", linestyle="--", linewidth=0.5)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("位置偏差 (μm)")
    ax.set_title("各轴位置偏差 vs 时间")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (b) 总漂移距离
    ax = axes[0][1]
    ax.plot(t, [d * 1000 for d in drift], color="red", linewidth=1.2)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("总漂移 (μm)")
    ax.set_title(f"末端总漂移 = {drift[-1]*1000:.3f} μm")
    ax.grid(True, alpha=0.3)

    # (c) 漂移方向 XY
    ax = axes[1][0]
    sc = ax.scatter(dx, dy, c=t, cmap="plasma", s=2, alpha=0.8)
    ax.set_xlabel("ΔX (mm)")
    ax.set_ylabel("ΔY (mm)")
    ax.set_title(f"XY 平面漂移 ({t[-1]:.0f}s)")
    ax.axhline(y=0, color="gray", ls="--", lw=0.5)
    ax.axvline(x=0, color="gray", ls="--", lw=0.5)
    ax.set_aspect("equal")
    plt.colorbar(sc, ax=ax, label="时间 (s)")

    # (d) 旋转偏差
    ax = axes[1][1]
    ax.plot(t, rfrob, color="purple", linewidth=0.8)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("旋转偏差 (Frobenius)")
    ax.set_title(f"旋转漂移 = {rfrob[-1]:.6f}")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig_path = out_dir / "svd_drift.png"
    plt.savefig(fig_path, dpi=150)
    print(f"保存: {fig_path}")

    # ── 图2: 漂移速率（差分） ──
    fig2, ax2 = plt.subplots(figsize=(12, 4))
    dt_avg = (t[-1] - t[0]) / (len(t) - 1)
    drift_rate = []
    for i in range(1, len(drift)):
        rate = (drift[i] - drift[i-1]) / (t[i] - t[i-1]) * 1e6  # nm/s
        drift_rate.append(rate)

    ax2.plot(t[1:], drift_rate, linewidth=0.5, alpha=0.7)
    ax2.axhline(y=0, color="gray", linestyle="--", linewidth=0.5)
    ax2.set_xlabel("时间 (s)")
    ax2.set_ylabel("瞬时漂移速率 (nm/s)")
    ax2.set_title("末端漂移速率 (差分)")
    ax2.grid(True, alpha=0.3)

    fig2_path = out_dir / "svd_drift_rate.png"
    plt.savefig(fig2_path, dpi=150)
    print(f"保存: {fig2_path}")

    # ── 终端统计 ──
    print(f"\n漂移统计 ({t[-1]:.0f}s):")
    print(f"  末端位置漂移: {drift[-1]*1000:.3f} μm")
    print(f"    X: {dx[-1]*1000:.3f} μm")
    print(f"    Y: {dy[-1]*1000:.3f} μm")
    print(f"    Z: {dz[-1]*1000:.3f} μm")
    print(f"  旋转漂移:     {rfrob[-1]:.6f} (Frobenius)")
    print(f"  平均漂移速率: {drift[-1]/t[-1]*1e6:.2f} nm/s")


if __name__ == "__main__":
    main()
