"""
ROCOS 坐标系管理组件组。

提供工具坐标系 (Tool Frame) 和工件坐标系 (Object Frame) 的
查询、设置、激活和删除功能。
对应 /api/robot/tool_frame, /api/robot/object_frame 等端点。
"""

from langflow.custom import Component
from langflow.io import Output, StrInput, FloatInput
from langflow.schema import Data
from .rocos_robot_state import _rocos_get, _rocos_post


# ---- 工具坐标系 ----

class GetToolFrames(Component):
    """获取所有工具坐标系名称列表"""

    display_name: str = "获取工具坐标系列表"
    description: str = (
        "获取 ROCOS 机器人所有已定义的工具坐标系名称列表，以及当前激活的工具坐标系名称。"
        "Use this to list available tool frames before activating one."
    )
    icon: str = "List"
    name: str = "get_tool_frames"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Frames List", name="frames", method="fetch_tool_frames"),
    ]

    def fetch_tool_frames(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/tool_frames")
        if result.get("success"):
            data = result.get("data", {})
            names = data.get("names", [])
            active = data.get("active", "")
            text = f"工具坐标系: {names}\n当前激活: {active or '(无)'}"
        else:
            text = f"❌ {result.get('message', '查询失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class SetToolFrame(Component):
    """设置或新增工具坐标系"""

    display_name: str = "设置工具坐标系"
    description: str = (
        "设置或新增一个命名的工具坐标系 (Tool Frame / TCP)。\n"
        "参数 pose (x,y,z, qx,qy,qz,qw) 定义工具相对于法兰的位姿偏移。"
        "使用此后，可调用 '激活工具坐标系' 来启用它。\n"
        "Use this to define a new TCP (Tool Center Point) offset from the flange."
    )
    icon: str = "Crosshair"
    name: str = "set_tool_frame"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="name",
            display_name="坐标系名称",
            info="工具坐标系名称（字母/数字/下划线/短横/点号）",
            value="tcp_default",
        ),
        StrInput(
            name="pos_x", display_name="位置 X (m)", value="0.0",
        ),
        StrInput(
            name="pos_y", display_name="位置 Y (m)", value="0.0",
        ),
        StrInput(
            name="pos_z", display_name="位置 Z (m)", value="0.2",
        ),
        StrInput(
            name="ori_x", display_name="Orientation X", value="0.0",
        ),
        StrInput(
            name="ori_y", display_name="Orientation Y", value="0.0",
        ),
        StrInput(
            name="ori_z", display_name="Orientation Z", value="0.0",
        ),
        StrInput(
            name="ori_w", display_name="Orientation W", value="1.0",
        ),
    ]

    outputs = [
        Output(display_name="Frame Result", name="frame_result", method="set_frame"),
    ]

    def set_frame(self) -> Data:
        try:
            frame = {
                "position": {
                    "x": float(self.pos_x),
                    "y": float(self.pos_y),
                    "z": float(self.pos_z),
                },
                "orientation": {
                    "x": float(self.ori_x),
                    "y": float(self.ori_y),
                    "z": float(self.ori_z),
                    "w": float(self.ori_w),
                },
            }
        except ValueError:
            return Data(text="❌ 参数错误: 位姿数值格式不正确", data=None)

        result = _rocos_post(self.base_url, "/api/robot/tool_frame", {
            "name": self.name,
            "frame": frame,
        })
        if result.get("success"):
            text = f"✅ 工具坐标系 '{self.name}' 已设置"
        else:
            text = f"❌ 设置失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class ActivateToolFrame(Component):
    """激活指定工具坐标系"""

    display_name: str = "激活工具坐标系"
    description: str = (
        "激活指定的工具坐标系。激活后，所有运动的目标位姿都将相对于此工具坐标系。"
        "Use this to switch the active TCP for subsequent motion commands."
    )
    icon: str = "CheckSquare"
    name: str = "activate_tool_frame"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="name",
            display_name="坐标系名称",
            info="要激活的工具坐标系名称",
            value="tcp_default",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="activate"),
    ]

    def activate(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/active_tool_frame", {
            "name": self.name,
        })
        if result.get("success"):
            text = f"✅ 已激活工具坐标系: {self.name}"
        else:
            text = f"❌ 激活失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


# ---- 工件坐标系 ----

class GetObjectFrames(Component):
    """获取所有工件坐标系名称列表"""

    display_name: str = "获取工件坐标系列表"
    description: str = (
        "获取 ROCOS 机器人所有已定义的工件坐标系名称列表，以及当前激活的工件坐标系名称。"
        "Use this to list available object/workpiece frames."
    )
    icon: str = "List"
    name: str = "get_object_frames"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Frames List", name="frames", method="fetch_object_frames"),
    ]

    def fetch_object_frames(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/object_frames")
        if result.get("success"):
            data = result.get("data", {})
            names = data.get("names", [])
            active = data.get("active", "")
            text = f"工件坐标系: {names}\n当前激活: {active or '(无)'}"
        else:
            text = f"❌ {result.get('message', '查询失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class SetObjectFrame(Component):
    """设置或新增工件坐标系"""

    display_name: str = "设置工件坐标系"
    description: str = (
        "设置或新增一个命名的工件坐标系 (Object Frame / Workpiece Frame)。\n"
        "参数 pose (x,y,z, qx,qy,qz,qw) 定义工件相对于基座的位姿。"
        "Use this to define a workpiece/world offset coordinate system."
    )
    icon: str = "Box"
    name: str = "set_object_frame"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="name",
            display_name="坐标系名称",
            info="工件坐标系名称（字母/数字/下划线/短横/点号）",
            value="table_1",
        ),
        StrInput(name="pos_x", display_name="位置 X (m)", value="0.5"),
        StrInput(name="pos_y", display_name="位置 Y (m)", value="0.3"),
        StrInput(name="pos_z", display_name="位置 Z (m)", value="0.0"),
        StrInput(name="ori_x", display_name="Orientation X", value="0.0"),
        StrInput(name="ori_y", display_name="Orientation Y", value="0.0"),
        StrInput(name="ori_z", display_name="Orientation Z", value="0.0"),
        StrInput(name="ori_w", display_name="Orientation W", value="1.0"),
    ]

    outputs = [
        Output(display_name="Frame Result", name="frame_result", method="set_frame"),
    ]

    def set_frame(self) -> Data:
        try:
            frame = {
                "position": {
                    "x": float(self.pos_x), "y": float(self.pos_y), "z": float(self.pos_z),
                },
                "orientation": {
                    "x": float(self.ori_x), "y": float(self.ori_y),
                    "z": float(self.ori_z), "w": float(self.ori_w),
                },
            }
        except ValueError:
            return Data(text="❌ 参数错误: 位姿数值格式不正确", data=None)

        result = _rocos_post(self.base_url, "/api/robot/object_frame", {
            "name": self.name,
            "frame": frame,
        })
        if result.get("success"):
            text = f"✅ 工件坐标系 '{self.name}' 已设置"
        else:
            text = f"❌ 设置失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class ActivateObjectFrame(Component):
    """激活指定工件坐标系"""

    display_name: str = "激活工件坐标系"
    description: str = (
        "激活指定的工件坐标系。激活后，运动的目标位姿将相对于此工件坐标系解读。"
        "Use this to switch the active workpiece coordinate system."
    )
    icon: str = "CheckSquare"
    name: str = "activate_object_frame"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="name",
            display_name="坐标系名称",
            info="要激活的工件坐标系名称",
            value="table_1",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="activate"),
    ]

    def activate(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/active_object_frame", {
            "name": self.name,
        })
        if result.get("success"):
            text = f"✅ 已激活工件坐标系: {self.name}"
        else:
            text = f"❌ 激活失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)
