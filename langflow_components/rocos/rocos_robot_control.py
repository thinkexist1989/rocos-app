"""
ROCOS 机器人基础控制组件组。

提供使能/禁用机器人、设置工作模式等功能。
对应 POST /api/robot/enable, /api/robot/disable, /api/robot/workmode
"""

from langflow.custom import Component
from langflow.io import Output, StrInput, DropdownInput
from langflow.schema import Data
from .rocos_robot_state import _rocos_post

# ---- 工作模式枚举 ----

WORK_MODES = [
    "position",
    "ee_admit_teach",
    "jnt_admit_teach",
    "jnt_imp",
    "cart_imp",
]


class EnableRobot(Component):
    """全局使能机器人"""

    display_name: str = "使能机器人"
    description: str = (
        "全局使能 ROCOS 机器人。所有关节将上电进入伺服状态。"
        "必须在使用任何运动指令之前调用此操作。"
        "如果机器人已使能，重复调用会返回错误 code=-2017。"
        "Use this tool to enable/power on the robot before sending motion commands."
    )
    icon: str = "Zap"
    name: str = "enable_robot"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="enable"),
    ]

    def enable(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/enable", {})
        if result.get("success"):
            text = "✅ 机器人已使能"
        else:
            code = result.get("code", 0)
            msg = result.get("message", "")
            if code == -2017:
                text = f"⚠️ 机器人已处于使能状态 ({msg})"
            else:
                text = f"❌ 使能失败 (code={code}): {msg}"
        self.status = text[:100]
        return Data(text=text, data=result)


class DisableRobot(Component):
    """全局禁用机器人"""

    display_name: str = "禁用机器人"
    description: str = (
        "全局禁用 ROCOS 机器人（下电）。所有关节失去伺服，释放扭矩。"
        "Use this tool to power off the robot or emergency stop."
    )
    icon: str = "PowerOff"
    name: str = "disable_robot"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="disable"),
    ]

    def disable(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/disable", {})
        if result.get("success"):
            text = "✅ 机器人已禁用"
        else:
            text = f"⚠️ {result.get('message', '禁用失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class SetWorkMode(Component):
    """设置机器人工���模式"""

    display_name: str = "设置工作模式"
    description: str = (
        "设置 ROCOS 机器人的工作模式 (Work Mode)。支持的运行模式有:\n"
        "- position: 标准位置控制模式\n"
        "- ee_admit_teach: 末端导纳示教模式（拖拽示教）\n"
        "- jnt_admit_teach: 关节导纳示教模式\n"
        "- jnt_imp: 关节阻抗控制模式\n"
        "- cart_imp: 笛卡尔阻抗控制模式\n"
        "Use this tool to switch between position control, admittance teach, and impedance control."
    )
    icon: str = "Sliders"
    name: str = "set_work_mode"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        DropdownInput(
            name="mode",
            display_name="工作模式",
            info="选择机器人工作模式",
            options=WORK_MODES,
            value="position",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="set_mode"),
    ]

    def set_mode(self) -> Data:
        result = _rocos_post(
            self.base_url, "/api/robot/workmode",
            {"mode": self.mode}
        )
        if result == {} or result.get("success", True):
            text = f"✅ 工作模式已切换为: {self.mode}"
            result = {"success": True, "code": 0, "message": "ok", "data": {"mode": self.mode}}
        else:
            text = f"❌ 切换失败 (code={result.get('code')}): {result.get('message')}"
        self.status = text[:100]
        return Data(text=text, data=result)
