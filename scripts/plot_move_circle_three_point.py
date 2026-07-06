#!/usr/bin/env python3
"""MoveCircleThreePoint — 3D arc verification + visualization."""

import os, sys, csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

CSV_DIR = "/tmp/move_circle_three_point_csv/"
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "output", "move_circle_three_point")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def load_csv(filename: str) -> dict:
    fp = os.path.join(CSV_DIR, filename)
    if not os.path.exists(fp):
        print(f"[ERROR] {fp}"); sys.exit(1)
    out = {}
    with open(fp) as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                out.setdefault(k, []).append(float(v))
    return {k: np.array(v) for k, v in out.items()}


def reconstruct_s(data: dict) -> np.ndarray:
    """Reconstruct s from arc angle via atan2."""
    a = np.unwrap(np.arctan2(data['py'], data['px']))
    return (a - a[0]) / (a[-1] - a[0])


# ======================================================================
def plot_3d_arc(normal, pause_resume, pause_stop, start_pt, via_pt, goal_pt,
                center, radius, prefix):
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(normal['px'], normal['py'], normal['pz'], 'b-', lw=2.5, alpha=0.7, label='Normal')
    ax.plot(pause_resume['px'], pause_resume['py'], pause_resume['pz'],
            'r--', lw=1.5, alpha=0.8, label='Pause & Resume')
    ax.plot(pause_stop['px'], pause_stop['py'], pause_stop['pz'],
            'g:', lw=1.5, alpha=0.9, label='Pause & Stop')

    # three input points
    for pt, c, m, name in [(start_pt, 'lime', 'o', 'Start'),
                             (via_pt,   'cyan', '^', 'Via'),
                             (goal_pt,  'red',  's', 'Goal')]:
        ax.scatter([pt[0]], [pt[1]], [pt[2]], color=c, s=120, marker=m,
                   edgecolors='black', zorder=5, label=name)

    ax.scatter([center[0]], [center[1]], [center[2]], color='gray', s=80,
               marker='x', zorder=5, label='Center')

    th = np.linspace(0, 2*np.pi, 200)
    ax.plot(center[0]+radius*np.cos(th), center[1]+radius*np.sin(th),
            center[2]+0*th, 'gray', lw=0.5, alpha=0.3, label='Full circle')

    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    ax.set_title(f'MoveCircleThreePoint — R={radius:.1f}m, theta={np.pi:.1f}')
    ax.legend(fontsize=8); ax.set_box_aspect([1,1,0.5])

    out = os.path.join(OUTPUT_DIR, f"{prefix}3d_arc.png")
    fig.savefig(out, dpi=150); plt.close(fig)
    print(f"[OK]  Saved: {out}")


def plot_time_overlay(normal, pause_resume, pause_stop, prefix):
    fig, axes = plt.subplots(2, 1, figsize=(16, 9), sharex=True)
    fig.suptitle('Time-Domain Overlay', fontsize=14, fontweight='bold')
    for k, c in zip(['px','py','pz'], ['r','g','b']):
        axes[0].plot(normal['time'], normal[k], '-', c=c, lw=2.5, alpha=0.4, label=f'{k} norm')
        axes[0].plot(pause_resume['time'], pause_resume[k], '--', c=c, lw=1.2, alpha=0.9)
        axes[0].plot(pause_stop['time'], pause_stop[k], ':', c=c, lw=1.2, alpha=0.9)
    axes[0].set_ylabel('Position [m]'); axes[0].legend(fontsize=7, ncol=3); axes[0].grid(True, alpha=0.3)
    for k, c in zip(['qx','qy','qz','qw'], ['r','g','b','orange']):
        axes[1].plot(normal['time'], normal[k], '-', c=c, lw=2.5, alpha=0.4)
        axes[1].plot(pause_resume['time'], pause_resume[k], '--', c=c, lw=1.2, alpha=0.9)
        axes[1].plot(pause_stop['time'], pause_stop[k], ':', c=c, lw=1.2, alpha=0.9)
    axes[1].set_ylabel('Quaternion'); axes[1].set_xlabel('Time [s]'); axes[1].grid(True, alpha=0.3)
    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}time_overlay.png")
    fig.savefig(out, dpi=150); plt.close(fig)
    print(f"[OK]  Saved: {out}")


def plot_verification(normal, pause_resume, prefix):
    s_norm = reconstruct_s(normal)
    s_pr   = reconstruct_s(pause_resume)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Phase Space Verification', fontsize=14, fontweight='bold')
    for j, (k, c) in enumerate(zip(['px','py','pz'], ['r','g','b'])):
        ax = axes[0][j]
        ax.plot(s_norm, normal[k], '-', c=c, lw=2.5, alpha=0.5)
        ax.plot(s_pr, pause_resume[k], '--', c=c, lw=1.2, alpha=0.9)
        ax.set_xlabel('s'); ax.set_ylabel(k); ax.set_title(f'{k} vs s'); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}phase_space.png")
    fig.savefig(out, dpi=150); plt.close(fig)
    print(f"[OK]  Saved: {out}")

    # Residual
    fig, ax = plt.subplots(figsize=(14, 5))
    fig.suptitle('Residual |pr - norm|', fontsize=14, fontweight='bold')
    for k, c in zip(['px','py','pz'], ['r','g','b']):
        err = np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k]))
        mx = np.max(err)
        ax.plot(s_pr, err, color=c, lw=0.8, alpha=0.8, label=f'{k} (max={mx:.1e})')
    ax.set_yscale('log'); ax.set_xlabel('s'); ax.set_ylabel('|diff|')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}residual.png")
    fig.savefig(out, dpi=150); plt.close(fig)
    print(f"[OK]  Saved: {out}")


def check_via(data, via_pt, label):
    """Verify the trajectory passes through the via point."""
    pos = np.column_stack([data['px'], data['py'], data['pz']])
    dists = np.linalg.norm(pos - via_pt, axis=1)
    min_dist = np.min(dists)
    idx = np.argmin(dists)
    s_at_via = reconstruct_s(data)[idx]
    print(f"  [{label}] Via check: min distance={min_dist:.2e}m at s={s_at_via:.4f}")
    return min_dist


def check_radius(data, center, radius, label):
    pos = np.column_stack([data['px'], data['py'], data['pz']])
    dev = np.max(np.abs(np.linalg.norm(pos - center, axis=1) - radius))
    print(f"  [{label}] Radius check: R={radius:.1f}, max dev={dev:.2e}m")
    return dev


def process(direction, label, start_pt, via_pt, goal_pt):
    prefix = f"{direction}_"
    normal       = load_csv(f"{prefix}normal.csv")
    pause_resume = load_csv(f"{prefix}pause_resume.csv")
    pause_stop   = load_csv(f"{prefix}pause_stop.csv")

    # Estimate center from normal trajectory
    pos_n = np.column_stack([normal['px'], normal['py'], normal['pz']])
    center_est = np.mean(pos_n[[0, len(pos_n)//2, -1]], axis=0)
    # For a half-circle: center = midpoint of start and goal
    center = (start_pt + goal_pt) / 2.0
    radius = np.linalg.norm(start_pt - center)

    print(f"\n[{label}]")
    print(f"  Normal:       {len(normal['step']):5d} pts, "
          f"final=({normal['px'][-1]:.4f},{normal['py'][-1]:.4f},{normal['pz'][-1]:.4f})")
    print(f"  Pause&Resume:  {len(pause_resume['step']):5d} pts, "
          f"final=({pause_resume['px'][-1]:.4f},{pause_resume['py'][-1]:.4f},{pause_resume['pz'][-1]:.4f})")
    print(f"  Pause&Stop:    {len(pause_stop['step']):5d} pts, "
          f"final=({pause_stop['px'][-1]:.4f},{pause_stop['py'][-1]:.4f},{pause_stop['pz'][-1]:.4f})")

    # Verify: passes through via point
    check_via(normal, via_pt, "Normal")
    check_radius(normal, center, radius, "Normal")
    check_radius(pause_resume, center, radius, "Pause&Resume")
    check_radius(pause_stop, center, radius, "Pause&Stop")

    print("[..] 3D arc...")
    plot_3d_arc(normal, pause_resume, pause_stop, start_pt, via_pt, goal_pt,
                center, radius, prefix)
    print("[..] Time overlay...")
    plot_time_overlay(normal, pause_resume, pause_stop, prefix)
    print("[..] Verification...")
    plot_verification(normal, pause_resume, prefix)

    s_norm = reconstruct_s(normal); s_pr = reconstruct_s(pause_resume)
    max_pos = max(np.max(np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k])))
                  for k in ['px','py','pz'])
    print(f"  Max position residual: {max_pos:.2e}")


def main():
    print("=" * 65)
    print("  MoveCircleThreePoint — 3-Point Arc Verification")
    print("=" * 65)

    # Forward: Start(1,0,0) Via(0,1,0) Goal(-1,0,0) — CCW upper half
    process("forward", "Forward: (1,0)->(-1,0) via (0,1)",
            np.array([1.,0.,0.]), np.array([0.,1.,0.]), np.array([-1.,0.,0.]))

    # Reverse: Start(-1,0,0) Via(0,1,0) Goal(1,0,0) — CCW upper half continued
    process("reverse", "Reverse: (-1,0)->(1,0) via (0,1)",
            np.array([-1.,0.,0.]), np.array([0.,1.,0.]), np.array([1.,0.,0.]))

    print(f"\n{'='*65}\n  Done! See: {OUTPUT_DIR}\n{'='*65}")


if __name__ == '__main__':
    main()
