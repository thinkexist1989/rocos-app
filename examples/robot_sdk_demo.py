#!/usr/bin/env python3
"""SDK example: query state, MoveJ out and back. Run with --help for options."""
import argparse

try:
    from .rocos import Robot
except ImportError:
    from rocos import Robot


def main():
    parser = argparse.ArgumentParser(description="ROCOS Python SDK 仿真往返示例")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--read-only", action="store_true", help="只查询，不申请控制权或运动")
    args = parser.parse_args()

    robot = Robot(args.host, port=args.port)
    if args.read_only:
        print("关节位置 [rad]：", robot.get_joints())
        print("TCP 位姿：", robot.get_pose())
        return

    # This demo moves only a simulation; the Robot client also supports real hardware.
    state = robot.get()
    if state["hw_state"]["hw_type"] != 1 or state["motion_busy"]:
        raise RuntimeError("请在空闲的 HardwareSim 仿真上运行此往返示例")
    with robot:
        robot.enable()
        original = robot.get_joints()
        target = original.copy()
        target[0] += 0.03
        try:
            robot.movej(target, velocity=0.1, acceleration=0.3)
            print("到达目标 [rad]：", robot.get_joints())
            robot.movej(original, velocity=0.1, acceleration=0.3)
            print("已回位 [rad]：", robot.get_joints())
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
