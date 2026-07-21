#!/usr/bin/env python3
"""绘制笛卡尔点动偏轴漂移 + 雅可比条件数"""
import csv, math, sys
from pathlib import Path

def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "build/offaxis_drift.csv"
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("build/offaxis_drift_plots")
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})

    t  = [r["t"]  for r in rows]
    dx = [r["dx"] for r in rows]  # mm
    dy = [r["dy"] * 1000 for r in rows]  # μm
    dz = [r["dz"] * 1000 for r in rows]  # μm
    cond = [r["cond_J"] for r in rows]

    print(f"Points: {len(rows)}, duration: {t[-1]:.1f}s")
    print(f"X travel: {dx[-1]:.3f} mm")
    print(f"Y max deviation: {max(abs(v) for v in dy):.3f} μm")
    print(f"Z max deviation: {max(abs(v) for v in dz):.3f} μm")
    print(f"Cond(J) range: {min(cond):.1f} ~ {max(cond):.1f}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("MoveJog BASE_X — Off-axis Drift (Planner Level)", fontsize=14)

    # 上: Y, Z 偏轴漂移 vs 时间
    ax = axes[0]
    ax.plot(t, dy, linewidth=0.6, label="dY (off-axis)", alpha=0.9)
    ax.plot(t, dz, linewidth=0.6, label="dZ (off-axis)", alpha=0.9)
    ax.axhline(y=0, color="gray", ls="--", lw=0.5)
    ax.set_ylabel("deviation (μm)")
    ax.set_title(f"Y/Z off-axis drift over {t[-1]:.0f}s X-jog")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 下: 雅可比条件数
    ax = axes[1]
    ax.plot(t, cond, linewidth=0.5, color="darkred")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("cond(J)")
    ax.set_title("Jacobian condition number")
    ax.grid(True, alpha=0.3)
    # 标注奇异区域
    ax.axhline(y=100, color="orange", ls="--", lw=0.5, alpha=0.5, label="well-conditioned")
    ax.legend()

    # Z vs X 行程
    fig2, ax2 = plt.subplots(figsize=(10, 5))
    ax2.plot(dx, dz, linewidth=0.6, color="steelblue")
    ax2.set_xlabel("X travel (mm)")
    ax2.set_ylabel("Z deviation (μm)")
    ax2.set_title(f"Z drift vs X travel (slope = {dz[-1]/dx[-1]:.4f} μm/mm)")
    ax2.grid(True, alpha=0.3)

    # 添加线性拟合
    if len(dx) > 10:
        n = len(dx)
        sx = sum(dx); sy = sum(dz)
        sxx = sum(x*x for x in dx); sxy = sum(dx[i]*dz[i] for i in range(n))
        slope = (n*sxy - sx*sy) / (n*sxx - sx*sx)
        ax2.plot([dx[0], dx[-1]], [slope*dx[0], slope*dx[-1]],
                 '--', color='red', linewidth=1, label=f'linear fit: {slope:.6f} μm/mm')
        ax2.legend()

    plt.tight_layout()
    fig.savefig(out_dir / "offaxis_drift.png", dpi=150)
    fig2.savefig(out_dir / "offaxis_drift_vs_x.png", dpi=150)
    print(f"\nPlots: {out_dir}/")


if __name__ == "__main__":
    main()
