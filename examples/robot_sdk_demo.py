#!/usr/bin/env python3
"""Common SDK calls: read, MoveJ, and MoveL demos, without HTTP plumbing."""
import argparse
import math

try:
    from .rocos import Robot
except ImportError:
    from rocos import Robot


TALON_MOVEL_START = (0.2, -0.4, 0.3, -0.55, 0.2, 0.4, 0.1)


def demo_read(robot):
    """Read-only: no control session is needed."""
    print("关节位置 [rad]：", robot.getjoints())
    print("关节速度 [rad/s]：", robot.get_joint_velocities())
    print("法兰位姿：", robot.getflange())
    print("TCP 位姿：", robot.gettool())
    print("是否运动中：", robot.is_moving())


def demo_movej(robot, joint=0, delta=0.03):
    """Move one joint relative to the starting position, then return."""
    original = robot.getjoints()
    if not 0 <= joint < len(original):
        raise ValueError("joint 索引超出关节范围")
    target = original.copy()
    target[joint] += delta
    robot.movej(target, velocity=0.1, acceleration=0.3)
    print("MoveJ 到达目标 [rad]：", robot.getjoints())
    robot.movej(original, velocity=0.1, acceleration=0.3)
    print("MoveJ 已回位 [rad]：", robot.getjoints())


def demo_movel(robot, dz=-0.005, prepare_joints=TALON_MOVEL_START):
    """MoveJ to a known preparation pose, MoveL out/back, then restore joints.

    The default preparation pose is for this project's seven-axis Talon model.
    Supply prepare_joints for a different robot. Legacy MoveL targets the flange.
    """
    original = robot.getjoints()
    if len(prepare_joints) != len(original):
        raise ValueError("默认 MoveL 预备姿态适用于 7 轴 Talon；请提供匹配的 prepare_joints")
    if not math.isfinite(dz) or dz == 0:
        raise ValueError("dz 必须是非零有限数，单位 m")
    robot.movej(prepare_joints, velocity=0.3, acceleration=0.6)
    start = robot.getflange()
    target = start.translated(dz=dz)
    robot.movel(target, velocity=0.01, acceleration=0.05, timeout=30)
    print("MoveL 到达目标：", robot.getflange())
    robot.movel(start, velocity=0.01, acceleration=0.05, timeout=30)
    print("MoveL 法兰已回位：", robot.getflange())
    robot.movej(original, velocity=0.3, acceleration=0.6)
    print("MoveL 示例已恢复初始关节位置：", robot.getjoints())


def main():
    parser = argparse.ArgumentParser(description="ROCOS Python SDK 常用函数示例")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--read-only", action="store_true", help="只查询，不申请控制权或运动")
    parser.add_argument("--demo", choices=("read", "movej", "movel", "all"), default="movej")
    parser.add_argument("--joint", type=int, default=0)
    parser.add_argument("--joint-delta", type=float, default=0.03, help="MoveJ 相对角度，rad")
    parser.add_argument("--dz", type=float, default=-0.005, help="MoveL 法兰沿基坐标 Z 的位移，m")
    parser.add_argument("--prepare-joints", type=float, nargs="+", default=TALON_MOVEL_START,
                        help="MoveL 前的关节预备姿态，rad；默认适用于 7 轴 Talon")
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.joint_delta, args.dz, *args.prepare_joints)):
        parser.error("运动参数必须是有限数值")

    robot = Robot(args.host, port=args.port)
    selected = "read" if args.read_only else args.demo
    if selected == "read":
        demo_read(robot)
        return

    # This demo moves only a simulation; the Robot client also supports real hardware.
    state = robot.get()
    if state["hw_state"]["hw_type"] != 1 or state["motion_busy"]:
        raise RuntimeError("请在空闲的 HardwareSim 仿真上运行此往返示例")
    with robot:
        robot.enable()
        try:
            demo_read(robot)
            if selected in ("movej", "all"):
                demo_movej(robot, args.joint, args.joint_delta)
            if selected in ("movel", "all"):
                demo_movel(robot, args.dz, args.prepare_joints)
        finally:
            robot.stop()
            if not state["is_enabled"]:
                robot.disable()
            if state["work_mode"] != "position":
                robot.set_work_mode(state["work_mode"])
                if not state["control_active"]:
                    robot.stop()


if __name__ == "__main__":
    main()
