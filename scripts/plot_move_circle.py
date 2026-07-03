#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveCircle Cartesian arc trajectory — 3D visualization + verification.
CSV: step,time,px,py,pz,qx,qy,qz,qw
"""

import os, sys, csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d

CSV_DIR = "/tmp/move_circle_csv/"
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "move_circle")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def load_csv(filename: str) -> dict:
    filepath = os.path.join(CSV_DIR, filename)
    if not os.path.exists(filepath):
        print(f"[ERROR] File not found: {filepath}")
        sys.exit(1)
    out = {}
    with open(filepath, 'r') as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                out.setdefault(k, []).append(float(v))
    for k in out:
        out[k] = np.array(out[k])
    return out


def reconstruct_s(data: dict) -> np.ndarray:
    """Reconstruct normalized progress s from arc angle."""
    px, py = data['px'], data['py']
    # Angle at each point atan2(y, x), with start at (R,0) → angle=0
    angles = np.arctan2(py, px)
    # Unwrap to handle crossing ±π
    angles = np.unwrap(angles)
    # Normalize: s = (angle - angle_start) / (angle_end - angle_start)
    return (angles - angles[0]) / (angles[-1] - angles[0])


# ======================================================================
# 3D 圆弧轨迹可视化
# ======================================================================

def plot_3d_arc(normal: dict, pause_resume: dict, pause_stop: dict,
                center: np.ndarray, radius: float, prefix: str):
    """3D visualization of the circular arc trajectory."""
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111, projection='3d')

    # Normal trajectory (ground truth arc)
    ax.plot(normal['px'], normal['py'], normal['pz'], 'b-',
            linewidth=2.5, alpha=0.7, label='Normal (ground truth)')

    # Pause&Resume — dashed overlay
    ax.plot(pause_resume['px'], pause_resume['py'], pause_resume['pz'], 'r--',
            linewidth=1.5, alpha=0.8, label='Pause & Resume')

    # Pause&Stop — dotted, truncated
    ax.plot(pause_stop['px'], pause_stop['py'], pause_stop['pz'], 'g:',
            linewidth=1.5, alpha=0.9, label='Pause & Stop')

    # Start/end markers
    ax.scatter([normal['px'][0]],  [normal['py'][0]],  [normal['pz'][0]],
               color='lime', s=120, marker='o', edgecolors='black', zorder=5, label='Start')
    ax.scatter([normal['px'][-1]], [normal['py'][-1]], [normal['pz'][-1]],
               color='red',  s=120, marker='s', edgecolors='black', zorder=5, label='Goal')
    ax.scatter([center[0]], [center[1]], [center[2]],
               color='gray', s=80, marker='x', zorder=5, label='Center')

    # Theoretical circle for reference
    theta_line = np.linspace(0, 2*np.pi, 200)
    ax.plot(center[0] + radius*np.cos(theta_line),
            center[1] + radius*np.sin(theta_line),
            center[2] + 0*theta_line,
            'gray', linewidth=0.5, alpha=0.3, label='Full circle (reference)')

    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    ax.set_title(f'MoveCircle — 3D Arc Trajectory\nR={radius:.1f}m')
    ax.legend(loc='upper left', fontsize=8)
    ax.set_box_aspect([1, 1, 0.5])

    out = os.path.join(OUTPUT_DIR, f"{prefix}3d_arc.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 2D 时域图
# ======================================================================

def plot_time_series(normal: dict, pause_resume: dict, pause_stop: dict, prefix: str):
    fig, axes = plt.subplots(2, 1, figsize=(16, 9), sharex=True)
    fig.suptitle('MoveCircle — Time-Domain Overlay', fontsize=14, fontweight='bold')

    # Position
    for k, c in zip(['px','py','pz'], ['r','g','b']):
        axes[0].plot(normal['time'], normal[k], '-', color=c, linewidth=2.5, alpha=0.4, label=f'{k} norm')
        axes[0].plot(pause_resume['time'], pause_resume[k], '--', color=c, linewidth=1.2, alpha=0.9)
        axes[0].plot(pause_stop['time'], pause_stop[k], ':', color=c, linewidth=1.2, alpha=0.9)
    axes[0].set_ylabel('Position [m]')
    axes[0].legend(fontsize=7, ncol=3, loc='lower right')
    axes[0].grid(True, alpha=0.3)

    # Quaternion
    for k, c in zip(['qx','qy','qz','qw'], ['r','g','b','orange']):
        axes[1].plot(normal['time'], normal[k], '-', color=c, linewidth=2.5, alpha=0.4)
        axes[1].plot(pause_resume['time'], pause_resume[k], '--', color=c, linewidth=1.2, alpha=0.9)
        axes[1].plot(pause_stop['time'], pause_stop[k], ':', color=c, linewidth=1.2, alpha=0.9)
    axes[1].set_ylabel('Quaternion')
    axes[1].set_xlabel('Time [s]')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}time_overlay.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 验证图：Phase space + Residual
# ======================================================================

def plot_verification(normal: dict, pause_resume: dict, prefix: str):
    s_norm = reconstruct_s(normal)
    s_pr   = reconstruct_s(pause_resume)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('MoveCircle — Phase Space & Residual Verification',
                 fontsize=14, fontweight='bold')

    # Phase space: pos vs s
    for j, (k, c) in enumerate(zip(['px','py','pz'], ['r','g','b'])):
        ax = axes[0][j]
        ax.plot(s_norm, normal[k], '-', color=c, linewidth=2.5, alpha=0.5)
        ax.plot(s_pr, pause_resume[k], '--', color=c, linewidth=1.2, alpha=0.9)
        ax.set_xlabel('s'); ax.set_ylabel(k)
        ax.set_title(f'{k} vs s')
        ax.grid(True, alpha=0.3)

    # Phase space: quat vs s
    for j, (k, c) in enumerate(zip(['qx','qy','qz','qw'], ['r','g','b','orange'])):
        ax = axes[1][j] if j < 3 else axes[1][2]
        ax.plot(s_norm, normal[k], '-', color=c, linewidth=2.5, alpha=0.5)
        ax.plot(s_pr, pause_resume[k], '--', color=c, linewidth=1.2, alpha=0.9)
        ax.set_xlabel('s'); ax.set_ylabel(k)
        ax.set_title(f'{k} vs s')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}phase_space.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")

    # Residual plot
    fig, ax = plt.subplots(figsize=(14, 5))
    fig.suptitle('MoveCircle — Position & Quaternion Residual |pr - norm|', fontsize=14, fontweight='bold')

    for k, c in zip(['px','py','pz'], ['r','g','b']):
        err = np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k]))
        ax.plot(s_pr, err, color=c, linewidth=0.8, alpha=0.8, label=f'{k}')
    for k, ls in zip(['qx','qy','qz','qw'], ['-','--',':','-.']):
        err = np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k]))
        ax.plot(s_pr, err, color='gray', linestyle=ls, linewidth=0.6, alpha=0.5, label=f'{k}')
    ax.set_yscale('log')
    ax.set_xlabel('s'); ax.set_ylabel('|diff|')
    ax.legend(fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}residual.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# Radius check: verify all points are on the circle
# ======================================================================

def check_radius(data: dict, center: np.ndarray, radius: float, label: str):
    positions = np.column_stack([data['px'], data['py'], data['pz']])
    dists = np.linalg.norm(positions - center, axis=1)
    max_dev = np.max(np.abs(dists - radius))
    print(f"  [{label}] Radius check: R={radius:.1f}, max deviation={max_dev:.2e}m")
    return max_dev


# ======================================================================
# main
# ======================================================================

def process_direction(direction: str, label: str, center: np.ndarray, radius: float):
    prefix = f"{direction}_"
    normal       = load_csv(f"{prefix}normal.csv")
    pause_resume = load_csv(f"{prefix}pause_resume.csv")
    pause_stop   = load_csv(f"{prefix}pause_stop.csv")

    print(f"\n[{label}]")
    print(f"  Normal:       {len(normal['step']):5d} pts, "
          f"final=({normal['px'][-1]:.4f},{normal['py'][-1]:.4f},{normal['pz'][-1]:.4f})")
    print(f"  Pause&Resume:  {len(pause_resume['step']):5d} pts, "
          f"final=({pause_resume['px'][-1]:.4f},{pause_resume['py'][-1]:.4f},{pause_resume['pz'][-1]:.4f})")
    print(f"  Pause&Stop:    {len(pause_stop['step']):5d} pts, "
          f"final=({pause_stop['px'][-1]:.4f},{pause_stop['py'][-1]:.4f},{pause_stop['pz'][-1]:.4f})")

    # Radius verification
    check_radius(normal, center, radius, "Normal")
    check_radius(pause_resume, center, radius, "Pause&Resume")
    check_radius(pause_stop, center, radius, "Pause&Stop")

    # 3D arc
    print("[..] 3D arc plot...")
    plot_3d_arc(normal, pause_resume, pause_stop, center, radius, prefix)

    # Time series
    print("[..] Time-domain overlay...")
    plot_time_series(normal, pause_resume, pause_stop, prefix)

    # Verification
    print("[..] Verification...")
    plot_verification(normal, pause_resume, prefix)

    # Quantitative
    s_norm = reconstruct_s(normal)
    s_pr   = reconstruct_s(pause_resume)
    max_pos_err = max(np.max(np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k])))
                      for k in ['px','py','pz'])
    max_quat_err = max(np.max(np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k])))
                       for k in ['qx','qy','qz','qw'])
    print(f"  Max position residual:  {max_pos_err:.2e}")
    print(f"  Max quaternion residual: {max_quat_err:.2e}")


def main():
    print("=" * 65)
    print("  MoveCircle — 3D Arc Trajectory Verification")
    print("=" * 65)
    print(f"  CSV dir: {CSV_DIR}")
    print(f"  Output : {OUTPUT_DIR}")

    center = np.array([0.0, 0.0, 0.0])
    radius = 0.5

    # Forward: (0.5,0,0) → theta=+π → (-0.5,0,0)
    process_direction("forward", "Forward  theta=+pi  (0.5,0,0)->(-0.5,0,0)",
                      center, radius)

    # Reverse: (-0.5,0,0) → theta=-π → (0.5,0,0)
    process_direction("reverse", "Reverse  theta=-pi  (-0.5,0,0)->(0.5,0,0)",
                      center, radius)

    print()
    print("=" * 65)
    print("  Done! See:", OUTPUT_DIR)
    print("=" * 65)


if __name__ == '__main__':
    main()
