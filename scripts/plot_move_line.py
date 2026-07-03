#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveLine Cartesian trajectory visualization + verification.

CSV format: step,time,px,py,pz,qx,qy,qz,qw
"""

import os, sys, csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

CSV_DIR = "/tmp/move_line_csv/"
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "move_line")
os.makedirs(OUTPUT_DIR, exist_ok=True)

SCENARIOS = {
    "normal":       ("Normal",       "blue"),
    "pause_resume": ("Pause&Resume", "red"),
    "pause_stop":   ("Pause&Stop",   "green"),
}


def load_csv(filename: str) -> dict:
    filepath = os.path.join(CSV_DIR, filename)
    if not os.path.exists(filepath):
        print(f"[ERROR] File not found: {filepath}")
        print(f"  Run: ./build/bin/move_line_test")
        sys.exit(1)

    out = {'step': [], 'time': [], 'px': [], 'py': [], 'pz': [],
           'qx': [], 'qy': [], 'qz': [], 'qw': []}
    with open(filepath, 'r') as f:
        for row in csv.DictReader(f):
            for k in out:
                out[k].append(float(row[k]))
    for k in out:
        out[k] = np.array(out[k])
    return out


def reconstruct_s(data: dict, p_start: np.ndarray, p_goal: np.ndarray) -> np.ndarray:
    """从平移距离反推归一化进度 s"""
    total_dist = np.linalg.norm(p_goal - p_start)
    if total_dist < 1e-9:
        return np.zeros_like(data['time'])
    positions = np.column_stack([data['px'], data['py'], data['pz']])
    return np.linalg.norm(positions - p_start, axis=1) / total_dist


# ======================================================================
# 单场景图：位置(3x) + 四元数(4x)
# ======================================================================

def plot_single(data: dict, title: str, filename: str, p_goal: np.ndarray):
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    fig.suptitle(title, fontsize=14, fontweight='bold')
    t = data['time']

    # 位置
    for c, color in zip(['px','py','pz'], ['r','g','b']):
        axes[0].plot(t, data[c], color=color, linewidth=1.0, label=c)
    for i, v in enumerate(p_goal):
        axes[0].axhline(y=v, color=['r','g','b'][i], linestyle=':', alpha=0.3)
    axes[0].set_ylabel('Position [m]')
    axes[0].legend(loc='lower right')
    axes[0].grid(True, alpha=0.3)

    # 四元数
    for c, color in zip(['qx','qy','qz','qw'], ['r','g','b','orange']):
        axes[1].plot(t, data[c], color=color, linewidth=1.0, label=c)
    axes[1].set_ylabel('Quaternion')
    axes[1].set_xlabel('Time [s]')
    axes[1].legend(loc='lower right')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, filename)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 验证图 1：时域叠加 — 位置 + 四元数
# ======================================================================

def plot_time_overlay(normal: dict, pause_resume: dict, pause_stop: dict,
                      p_goal: np.ndarray, prefix: str = ""):
    fig, axes = plt.subplots(2, 1, figsize=(18, 10), sharex=True)
    fig.suptitle('Verification 1: Time-Domain Overlay', fontsize=14, fontweight='bold')

    pos_keys = ['px','py','pz']
    pos_colors = ['r','g','b']
    quat_keys = ['qx','qy','qz','qw']
    quat_colors = ['r','g','b','orange']

    # Position overlay
    ax = axes[0]
    for k, c in zip(pos_keys, pos_colors):
        ax.plot(normal['time'], normal[k], '-', color=c, linewidth=2.5, alpha=0.5,
                label=f'{k} (normal)')
        ax.plot(pause_resume['time'], pause_resume[k], '--', color=c,
                linewidth=1.2, alpha=0.9)
        ax.plot(pause_stop['time'], pause_stop[k], ':', color=c,
                linewidth=1.2, alpha=0.9)
    ax.set_ylabel('Position [m]')
    ax.legend(fontsize=7, ncol=3, loc='lower right')
    ax.grid(True, alpha=0.3)

    # Quaternion overlay
    ax = axes[1]
    for k, c in zip(quat_keys, quat_colors):
        ax.plot(normal['time'], normal[k], '-', color=c, linewidth=2.5, alpha=0.5,
                label=f'{k} (normal)')
        ax.plot(pause_resume['time'], pause_resume[k], '--', color=c,
                linewidth=1.2, alpha=0.9)
        ax.plot(pause_stop['time'], pause_stop[k], ':', color=c,
                linewidth=1.2, alpha=0.9)
    ax.set_ylabel('Quaternion')
    ax.set_xlabel('Time [s]')
    ax.legend(fontsize=7, ncol=4, loc='lower right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}verify1_time_overlay.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 验证图 2：相空间 p vs s — 空间路径是否重合
# ======================================================================

def plot_phase_space(normal: dict, pause_resume: dict,
                     p_start: np.ndarray, p_goal: np.ndarray, prefix: str = ""):
    s_norm = reconstruct_s(normal, p_start, p_goal)
    s_pr   = reconstruct_s(pause_resume, p_start, p_goal)

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Verification 2: Phase Space (value vs s)', fontsize=14, fontweight='bold')

    # Position vs s
    ax = axes[0][0]
    for k, c in zip(['px','py','pz'], ['r','g','b']):
        ax.plot(s_norm, normal[k], '-', color=c, linewidth=2.5, alpha=0.5)
        ax.plot(s_pr, pause_resume[k], '--', color=c, linewidth=1.2, alpha=0.9)
    s_line = np.linspace(0, 1, 100)
    for i, (k, c) in enumerate(zip(['px','py','pz'], ['r','g','b'])):
        ax.plot(s_line, p_start[i] + s_line * (p_goal[i] - p_start[i]),
                ':', color=c, linewidth=0.5, alpha=0.4)
    ax.set_xlabel('s'); ax.set_ylabel('Position [m]')
    ax.set_title('Position vs s (should overlap perfectly)')
    ax.grid(True, alpha=0.3)

    # Quaternion vs s
    ax = axes[0][1]
    for k, c in zip(['qx','qy','qz','qw'], ['r','g','b','orange']):
        ax.plot(s_norm, normal[k], '-', color=c, linewidth=2.5, alpha=0.5)
        ax.plot(s_pr, pause_resume[k], '--', color=c, linewidth=1.2, alpha=0.9)
    ax.set_xlabel('s'); ax.set_ylabel('Quaternion')
    ax.set_title('Quaternion vs s (Slerp path should overlap)')
    ax.grid(True, alpha=0.3)

    # Position residual
    ax = axes[1][0]
    for k, c in zip(['px','py','pz'], ['r','g','b']):
        err = np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k]))
        ax.plot(s_pr, err, color=c, linewidth=0.8, alpha=0.8, label=f'{k}')
    ax.set_yscale('log')
    ax.set_xlabel('s'); ax.set_ylabel('|pos_diff|')
    ax.set_title('Position residual |pr - norm|')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Quaternion residual
    ax = axes[1][1]
    for k, c in zip(['qx','qy','qz','qw'], ['r','g','b','orange']):
        err = np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k]))
        ax.plot(s_pr, err, color=c, linewidth=0.8, alpha=0.8, label=f'{k}')
    ax.set_yscale('log')
    ax.set_xlabel('s'); ax.set_ylabel('|quat_diff|')
    ax.set_title('Quaternion residual |pr - norm|')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}verify2_phase_space.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 验证图 3：Stop 截断验证
# ======================================================================

def plot_stop_truncation(normal: dict, pause_stop: dict,
                         p_start: np.ndarray, p_goal: np.ndarray, prefix: str = ""):
    s_norm = reconstruct_s(normal, p_start, p_goal)
    s_stop = reconstruct_s(pause_stop, p_start, p_goal)
    s_final = s_stop[-1]
    cutoff = np.searchsorted(s_norm, s_final)

    fig, axes = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(f'Verification 3: Stop Truncation (stop at s={s_final:.4f})',
                 fontsize=14, fontweight='bold')

    # Position
    ax = axes[0]
    for k, c in zip(['px','py','pz'], ['r','g','b']):
        ax.plot(normal['time'], normal[k], '-', color=c, linewidth=1.0, alpha=0.2)
        ax.plot(normal['time'][:cutoff], normal[k][:cutoff], '-',
                color=c, linewidth=2.0, alpha=0.8, label=f'{k} norm trunced')
        ax.plot(pause_stop['time'], pause_stop[k], '--', color=c,
                linewidth=1.2, alpha=0.9, label=f'{k} stop')
    ax.axvline(x=pause_stop['time'][-1], color='gray', linestyle=':', alpha=0.5)
    ax.set_ylabel('Position [m]')
    ax.legend(fontsize=7, ncol=3, loc='lower right')
    ax.grid(True, alpha=0.3)

    # Quaternion
    ax = axes[1]
    for k, c in zip(['qx','qy','qz','qw'], ['r','g','b','orange']):
        ax.plot(normal['time'], normal[k], '-', color=c, linewidth=1.0, alpha=0.2)
        ax.plot(normal['time'][:cutoff], normal[k][:cutoff], '-',
                color=c, linewidth=2.0, alpha=0.8)
        ax.plot(pause_stop['time'], pause_stop[k], '--', color=c,
                linewidth=1.2, alpha=0.9)
    ax.axvline(x=pause_stop['time'][-1], color='gray', linestyle=':', alpha=0.5)
    ax.set_ylabel('Quaternion')
    ax.set_xlabel('Time [s]')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{prefix}verify3_stop_truncation.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# main
# ======================================================================

def process_direction(direction: str, label: str, p_start: np.ndarray, p_goal: np.ndarray):
    prefix = f"{direction}_"
    normal       = load_csv(f"{prefix}normal.csv")
    pause_resume = load_csv(f"{prefix}pause_resume.csv")
    pause_stop   = load_csv(f"{prefix}pause_stop.csv")

    print(f"[{label}]")
    print(f"  Normal:       {len(normal['step']):5d} pts, "
          f"final=({normal['px'][-1]:.4f},{normal['py'][-1]:.4f},{normal['pz'][-1]:.4f})")
    print(f"  Pause&Resume:  {len(pause_resume['step']):5d} pts, "
          f"final=({pause_resume['px'][-1]:.4f},{pause_resume['py'][-1]:.4f},{pause_resume['pz'][-1]:.4f})")
    print(f"  Pause&Stop:    {len(pause_stop['step']):5d} pts, "
          f"final=({pause_stop['px'][-1]:.4f},{pause_stop['py'][-1]:.4f},{pause_stop['pz'][-1]:.4f})")

    print("[..] Single scenarios...")
    plot_single(normal,       f"{label} — Normal",         f"{prefix}01_normal.png",       p_goal)
    plot_single(pause_resume, f"{label} — Pause & Resume", f"{prefix}02_pause_resume.png", p_goal)
    plot_single(pause_stop,   f"{label} — Pause & Stop",   f"{prefix}03_pause_stop.png",   p_goal)

    print("[..] Verification...")
    plot_time_overlay(normal, pause_resume, pause_stop, p_goal, prefix)
    plot_phase_space(normal, pause_resume, p_start, p_goal, prefix)
    plot_stop_truncation(normal, pause_stop, p_start, p_goal, prefix)

    # 定量
    s_norm = reconstruct_s(normal, p_start, p_goal)
    s_pr   = reconstruct_s(pause_resume, p_start, p_goal)
    max_pos_err = max(np.max(np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k])))
                      for k in ['px','py','pz'])
    max_quat_err = max(np.max(np.abs(pause_resume[k] - np.interp(s_pr, s_norm, normal[k])))
                       for k in ['qx','qy','qz','qw'])
    print(f"  Max position residual:  {max_pos_err:.2e}")
    print(f"  Max quaternion residual: {max_quat_err:.2e}")
    print()


def main():
    print("=" * 65)
    print("  MoveLine — Cartesian Trajectory Verification")
    print("=" * 65)
    print(f"  CSV dir: {CSV_DIR}")
    print(f"  Output : {OUTPUT_DIR}")
    print()

    # 正向: (0,0,0) → (0.4,0.3,0.5), rot 0→60°
    process_direction("forward", "Forward [0,0,0]->[0.4,0.3,0.5]",
                      np.array([0.0, 0.0, 0.0]), np.array([0.4, 0.3, 0.5]))

    # 反向: (0.4,0.3,0.5) → (0,0,0), rot 60°→0
    process_direction("reverse", "Reverse [0.4,0.3,0.5]->[0,0,0]",
                      np.array([0.4, 0.3, 0.5]), np.array([0.0, 0.0, 0.0]))

    print("=" * 65)
    print("  Done! See:", OUTPUT_DIR)
    print("=" * 65)


if __name__ == '__main__':
    main()
