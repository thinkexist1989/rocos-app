"""
ROCOS 拖拽示教 (Jogging) 组件组。

提供启动/停止点动控制 (drag/jog) 的功能。支持关节方向、笛卡尔方向（工具/法兰/工件/基座坐标系）及零空间点动。
"""

from langflow.custom import Component
from langflow.io import Output, StrInput, DropdownInput, FloatInput
from langflow.schema import Data
from .rocos_robot_state import _rocos_post

# ---- 点动方向枚举 ----

JOG_FLAGS = [
    # 关节
    "J0", "J1", "J2", "J3", "J4", "J5", "J6",
    # 笛卡尔 - 工具坐标系
    "TOOL_X", "TOOL_Y", "TOOL_Z", "TOOL_ROLL", "TOOL_PITCH", "TOOL_YAW",
    # 笛卡尔 - 法兰坐标系
    "FLANGE_X", "FLANGE_Y", "FLANGE_Z", "FLANGE_ROLL", "FLANGE_PITCH", "FLANGE_YAW",
    # 笛卡尔 - 工件坐标系
    "OBJECT_X", "OBJECT_Y", "OBJECT_Z", "OBJECT_ROLL", "OBJECT_PITCH", "OBJECT_YAW",
    # 笛卡尔 - 基座坐标系
    "BASE_X", "BASE_Y", "BASE_Z", "BASE_ROLL", "BASE_PITCH", "BASE_YAW",
    # 零空间
    "NULLSPACE",
]

DIRECTIONS = ["POSITIVE", "NEGATIVE", "NONE"]


class JogStart(Component):
    """启动拖拽示教/点动"""

    display_name: str = "启动点动"
    description: str = (
        "启动 ROCOS 机器人点动 (Jogging / Drag)。\n"
        "flag 指定运动方向:\n"
        "  关节: J0~J6\n"
        "  笛卡尔: TOOL_X~TOOL_YAW (工具系), FLANGE_X~FLANGE_YAW (法兰系),\n"
        "          OBJECT_X~OBJECT_YAW (工件系), BASE_X~BASE_YAW (基座系)\n"
        "  NULLSPACE: 零空间点动（不改变末端位姿）\n"
        "direction: POSITIVE (正方向), NEGATIVE (负方向), NONE (停止分量)\n"
        "Use this tool to jog the robot in a specific direction or axis."
    )
    icon: str = "MousePointerClick"
    name: str = "jog_start"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        DropdownInput(
            name="flag",
            display_name="点动方向/轴",
            info="选择点动的目标轴或方向",
            options=JOG_FLAGS,
            value="J0",
        ),
        DropdownInput(
            name="direction",
            display_name="运动方向",
            info="POSITIVE=正方向, NEGATIVE=负方向, NONE=停止分量",
            options=DIRECTIONS,
            value="POSITIVE",
        ),
        FloatInput(
            name="max_speed",
            display_name="最大速度",
            info="最大点动速度",
            value=1.0,
        ),
        FloatInput(
            name="max_acceleration",
            display_name="最大加速度",
            info="最大点动加速度",
            value=2.0,
        ),
    ]

    outputs = [
        Output(display_name="Jog Result", name="jog_result", method="start_jog"),
    ]

    def start_jog(self) -> Data:
        result = _rocos_post(self.base_url, "/api/drag/start", {
            "flag": self.flag,
            "direction": self.direction,
            "max_speed": self.max_speed,
            "max_acceleration": self.max_acceleration,
        })
        if result.get("success"):
            text = f"✅ 点动已启动: {self.flag} {self.direction}"
        else:
            text = f"❌ 点动失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class JogStop(Component):
    """停止拖拽示教/点动"""

    display_name: str = "停止点动"
    description: str = (
        "停止 ROCOS 机器人所有正在进行的点动/拖拽运动。"
        "Use this to stop jogging immediately."
    )
    icon: str = "MousePointer"
    name: str = "jog_stop"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="stop_jog"),
    ]

    def stop_jog(self) -> Data:
        result = _rocos_post(self.base_url, "/api/drag/stop", {})
        if result.get("success"):
            text = "✅ 点动已停止"
        else:
            text = f"⚠️ {result.get('message', '停止点动失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)
