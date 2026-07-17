"""
ROCOS 运动控制组件组。

提供关节空间运动 (MoveJ)、笛卡尔直线运动 (MoveL)、停止/暂停/继续、
以及查询运动任务状态的功能。
"""

from langflow.custom import Component
from langflow.io import Output, StrInput, FloatInput
from langflow.schema import Data
from .rocos_client import _rocos_get, _rocos_post, parse_float_list

class MoveJ(Component):
    """关节空间运动 (MoveJ)"""

    display_name: str = "MoveJ 关节运动"
    description: str = (
        "执行关节空间运动 (MoveJ): 将机器人各关节移动到指定的目标角度。\n"
        "输入目标关节角度（弧度），数量需与当前机器人模型关节数一致，用逗号分隔。\n"
        "Use this tool to move the robot in joint space to specified joint angles."
    )
    icon: str = "Move"
    name: str = "move_j"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="joints",
            display_name="目标关节角度",
            info="目标关节角度 (弧度)，用逗号分隔，如: 0.1,0.2,0.3,0.4,0.5,0.6,0.7",
            value="0.1,0.2,0.3,0.4,0.5,0.6,0.7",
        ),
        FloatInput(
            name="speed",
            display_name="速度限制",
            info="速度缩放系数，默认 1.0",
            value=1.0,
        ),
        FloatInput(
            name="acceleration",
            display_name="加速度限制",
            info="加速度缩放系数，默认 2.0",
            value=2.0,
        ),
        FloatInput(
            name="jerk",
            display_name="加加速度限制",
            info="jerk 限制，默认 10.0",
            value=10.0,
        ),
    ]

    outputs = [
        Output(display_name="Motion Result", name="motion_result", method="execute_movej"),
    ]

    def execute_movej(self) -> Data:
        try:
            joint_list = parse_float_list(self.joints)
        except ValueError:
            return Data(text="❌ 参数错误: joints 格式不正确，需要逗号分隔的数值", data=None)

        result = _rocos_post(self.base_url, "/api/robot/movej", {
            "joints": joint_list,
            "velocity": self.speed,
            "acceleration": self.acceleration,
            "jerk": self.jerk,
        })
        if result.get("success"):
            text = f"✅ MoveJ 已执行 → 目标: {joint_list[:3]}... 状态: {result.get('data', {}).get('robot_state', 'RUNNING')}"
        else:
            code = result.get("code", 0)
            text = f"❌ MoveJ 失败 (code={code}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class MoveL(Component):
    """笛卡尔直线运动 (MoveL)"""

    display_name: str = "MoveL 直线运动"
    description: str = (
        "执行笛卡尔直线运动 (MoveL): 控制机器人末端沿直线移动到目标位姿。\n"
        "目标位姿由 position (x,y,z) 和 orientation (x,y,z,w) 四元数指定。\n"
        "Use this tool to move the robot end effector in a straight line to a Cartesian pose."
    )
    icon: str = "MoveDiagonal"
    name: str = "move_l"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="pos_x", display_name="位置 X (m)",
            info="目标 X 坐标", value="0.5",
        ),
        StrInput(
            name="pos_y", display_name="位置 Y (m)",
            info="目标 Y 坐标", value="0.0",
        ),
        StrInput(
            name="pos_z", display_name="位置 Z (m)",
            info="目标 Z 坐标", value="0.8",
        ),
        StrInput(
            name="ori_x", display_name="Orientation X",
            info="四元数 X 分量", value="0.0",
        ),
        StrInput(
            name="ori_y", display_name="Orientation Y",
            info="四元数 Y 分量", value="0.0",
        ),
        StrInput(
            name="ori_z", display_name="Orientation Z",
            info="四元数 Z 分量", value="0.0",
        ),
        StrInput(
            name="ori_w", display_name="Orientation W",
            info="四元数 W 分量", value="1.0",
        ),
        FloatInput(
            name="speed",
            display_name="速度限制",
            info="速度缩放系数", value=1.0,
        ),
        FloatInput(
            name="acceleration",
            display_name="加速度限制",
            info="加速度缩放系数", value=2.0,
        ),
        FloatInput(
            name="jerk",
            display_name="加加速度限制",
            info="jerk 限制", value=10.0,
        ),
        StrInput(
            name="tool_name",
            display_name="工具坐标系名称",
            info="可选。指定用于 MoveL 的工具坐标系名称，留空则使用当前激活工具系",
            value="",
            required=False,
        ),
    ]

    outputs = [
        Output(display_name="Motion Result", name="motion_result", method="execute_movel"),
    ]

    def execute_movel(self) -> Data:
        try:
            pose = {
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
            return Data(text="❌ 参数错误: 位姿参数格式不正确", data=None)

        body = {
            "pose": pose,
            "velocity": self.speed,
            "acceleration": self.acceleration,
            "jerk": self.jerk,
        }
        if self.tool_name:
            body["tool_name"] = self.tool_name.strip()

        result = _rocos_post(self.base_url, "/api/robot/movel", body)
        if result.get("success"):
            pos = pose["position"]
            text = f"✅ MoveL 已执行 → 目标: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})"
        else:
            code = result.get("code", 0)
            text = f"❌ MoveL 失败 (code={code}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class MotionStop(Component):
    """停止运动"""

    display_name: str = "停止运动"
    description: str = (
        "立即停止 ROCOS 机器人当前正在执行的任何运动。"
        "Use this tool to abruptly stop the robot's current motion."
    )
    icon: str = "StopCircle"
    name: str = "motion_stop"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="stop"),
    ]

    def stop(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/stop", {})
        if result.get("success"):
            text = "✅ 运动已停止"
        else:
            code = result.get("code", 0)
            msg = result.get("message", "")
            if code == -2307:
                text = f"⚠️ 已在停止状态 (code={code})"
            else:
                text = f"❌ 停止失败 (code={code}): {msg}"
        self.status = text[:100]
        return Data(text=text, data=result)


class MotionPause(Component):
    """暂停运动"""

    display_name: str = "暂停运动"
    description: str = (
        "暂停 ROCOS 机器人当前的运动。运动可随后通过继续指令恢复。"
        "Use this to temporarily pause motion, e.g. for inspection."
    )
    icon: str = "PauseCircle"
    name: str = "motion_pause"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="pause"),
    ]

    def pause(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/pause", {})
        if result.get("success"):
            text = "✅ 运动已暂停"
        else:
            text = f"⚠️ {result.get('message', '暂停失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class MotionResume(Component):
    """继续运动"""

    display_name: str = "继续运动"
    description: str = (
        "继续执行之前暂停的运动。"
        "Use this to resume a previously paused motion."
    )
    icon: str = "PlayCircle"
    name: str = "motion_resume"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="resume"),
    ]

    def resume(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/resume", {})
        if result.get("success"):
            text = "✅ 运动已继续"
        else:
            text = f"⚠️ {result.get('message', '继续失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class GetMotionStatus(Component):
    """查询运动任务状态"""

    display_name: str = "查询运动状态"
    description: str = (
        "查询 ROCOS 机器人的运动状态，包括 FSM 状态、是否正在运行、控制是否活跃。"
        "可选的 task_id 参数用于查询特定异步运动任务的详情。"
        "Use this tool to check if the robot is currently moving or to query a specific async motion task."
    )
    icon: str = "BarChart2"
    name: str = "get_motion_status"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="task_id",
            display_name="任务 ID",
            info="可选的异步任务 ID，留空则返回通用运动状态",
            value="",
            required=False,
        ),
    ]

    outputs = [
        Output(display_name="Status", name="status", method="fetch_status"),
    ]

    def fetch_status(self) -> Data:
        task_id = self.task_id.strip() if self.task_id else None
        result = _rocos_get(self.base_url, "/api/robot/move_status",
                            {"task_id": task_id} if task_id else None)
        if result.get("success"):
            data = result.get("data", {})
            text = (
                f"状态: {data.get('robot_state', 'UNKNOWN')}\n"
                f"运行中: {data.get('is_running', False)}, 控制活跃: {data.get('control_active', False)}"
            )
            if task_id:
                task = data.get("task")
                text += f"\n任务 {task_id}: {task if task else '无详情'}"
        else:
            text = f"❌ {result.get('message', '查询失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class WaitMove(Component):
    """等待当前运动完成"""

    display_name: str = "等待运动完成"
    description: str = (
        "等待 ROCOS 当前运动结束。后端会先延时 20ms，然后轮询状态机直到运动完成或错误。"
        "Use this after a motion command when the next step must wait for robot motion completion."
    )
    icon: str = "Timer"
    name: str = "wait_move"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="wait"),
    ]

    def wait(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/wait_move", {})
        if result.get("success"):
            text = f"✅ 当前运动已完成，状态: {result.get('data', {}).get('robot_state', 'UNKNOWN')}"
        else:
            text = f"❌ 等待失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)
