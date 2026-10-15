#!/usr/bin/env python3
"""Initialize Robot, then call its methods directly. Seven-axis Talon simulation.

python3 robot_class_demo.py 192.168.1.117:8080
"""
import sys

try:
    from .rocos import Robot
except ImportError:
    from rocos import Robot


if __name__ == "__main__":
    robot = Robot(sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1")
    initial = robot.get()
    if initial["hw_state"]["hw_type"] != 1 or robot.is_moving():
        raise RuntimeError("本示例请在空闲的七轴 Talon 仿真上运行")
    if len(robot.getjointposition()) != 7:
        raise RuntimeError("示例目标适用于七轴 Talon，请按自己的机器人修改目标")

    try:
        robot.enable()
        original = robot.getjointposition()
        print("全部关节：", original)
        print("关节 0：", robot.getjointposition(0))

        robot.movej([0.2, -0.4, 0.3, -0.55, 0.2, 0.4, 0.1], velocity=0.3, acceleration=0.6)

        flange = robot.getflange()
        print("法兰：", flange)
        print("TCP：", robot.gettool())
        robot.movel(flange.translated(dz=-0.005), velocity=0.01, acceleration=0.05)
        robot.movel(flange, velocity=0.01, acceleration=0.05)
        robot.movej(original, velocity=0.3, acceleration=0.6)
        print("已恢复关节位置：", robot.getjointposition())
    finally:
        try:
            robot.stop()
            if not initial["is_enabled"]:
                robot.disable()
            if initial["work_mode"] != "position":
                robot.set_work_mode(initial["work_mode"])
                if not initial["control_active"]:
                    robot.stop()
        finally:
            robot.close()
