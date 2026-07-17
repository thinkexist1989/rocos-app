"""
ROCOS Lua 脚本管理组件组。

提供上传、运行、暂停、继续、停止 Lua 脚本以及查询脚本状态的功能。
对应 /api/script/* 系列端点。
"""

from langflow.custom import Component
from langflow.io import Output, StrInput, IntInput
from langflow.schema import Data
from .rocos_robot_state import _rocos_get, _rocos_post


class UploadScript(Component):
    """上传并编译 Lua 脚本到内存"""

    display_name: str = "上传 Lua 脚本"
    description: str = (
        "上传 Lua 脚本源码到 ROCOS 机器人控制器内存中（不写入文件系统）。\n"
        "脚本编译通过后进入 LOADED 状态，随后可调用 '运行脚本' 来异步执行。\n"
        "脚本中可以调用 robot.MoveJ(), robot.MoveL(), robot.Sleep() 等内置函数。\n"
        "Use this to load a Lua script before executing it."
    )
    icon: str = "Upload"
    name: str = "upload_script"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="filename",
            display_name="脚本文件名",
            info="脚本文件名（仅用于标识，不实际写入文件）",
            value="task.lua",
        ),
        StrInput(
            name="source",
            display_name="Lua 源码",
            info="Lua 脚本源码，支持 robot.MoveJ, robot.MoveL, robot.Sleep 等 API",
            value='robot.Sleep(100)\n',
        ),
    ]

    outputs = [
        Output(display_name="Script Result", name="script_result", method="upload"),
    ]

    def upload(self) -> Data:
        result = _rocos_post(self.base_url, "/api/script/upload", {
            "filename": self.filename,
            "source": self.source,
        })
        if result.get("success"):
            data = result.get("data", {})
            text = (
                f"✅ 脚本已上传\n"
                f"  ID: {data.get('script_id', 'N/A')}\n"
                f"  状态: {data.get('state', 'UNKNOWN')}"
            )
        else:
            text = f"❌ 上传失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class RunScript(Component):
    """异步执行已加载的 Lua 脚本"""

    display_name: str = "运行 Lua 脚本"
    description: str = (
        "异步执行之前上传的 Lua 脚本。脚本在后台运行，可通过 '查询脚本状态' 追踪进度。"
        "Use this to start execution of a loaded Lua script."
    )
    icon: str = "Play"
    name: str = "run_script"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Script Result", name="script_result", method="run"),
    ]

    def run(self) -> Data:
        result = _rocos_post(self.base_url, "/api/script/run", {})
        if result.get("success"):
            data = result.get("data", {})
            text = (
                f"✅ 脚本已开始执行\n"
                f"  ID: {data.get('script_id', 'N/A')}\n"
                f"  状态: {data.get('state', 'RUNNING')}"
            )
        else:
            code = result.get("code", 0)
            text = f"❌ 运行失败 (code={code}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class PauseScript(Component):
    """暂停 Lua 脚本"""

    display_name: str = "暂停 Lua 脚本"
    description: str = (
        "暂停当前正在执行的 Lua 脚本及其活动运动。"
        "Use this to pause a running Lua script."
    )
    icon: str = "Pause"
    name: str = "pause_script"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Script Result", name="script_result", method="pause"),
    ]

    def pause(self) -> Data:
        result = _rocos_post(self.base_url, "/api/script/pause", {})
        if result.get("success"):
            data = result.get("data", {})
            text = f"✅ 脚本已请求暂停 → 状态: {data.get('state', 'PAUSING')}"
        else:
            text = f"⚠️ {result.get('message', '暂停失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class ResumeScript(Component):
    """继续 Lua 脚本"""

    display_name: str = "继续 Lua 脚本"
    description: str = (
        "继续执行之前暂停的 Lua 脚本及其活动运动。"
        "Use this to resume a paused Lua script."
    )
    icon: str = "SkipForward"
    name: str = "resume_script"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Script Result", name="script_result", method="resume"),
    ]

    def resume(self) -> Data:
        result = _rocos_post(self.base_url, "/api/script/resume", {})
        if result.get("success"):
            data = result.get("data", {})
            text = f"✅ 脚本已继续 → 状态: {data.get('state', 'RUNNING')}"
        else:
            code = result.get("code", 0)
            text = f"⚠️ 继续失败 (code={code}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class StopScript(Component):
    """停止 Lua 脚本"""

    display_name: str = "停止 Lua 脚本"
    description: str = (
        "停止当前正在执行的 Lua 脚本及其活动运动。"
        "Use this to stop a running or paused Lua script."
    )
    icon: str = "Square"
    name: str = "stop_script"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Script Result", name="script_result", method="stop"),
    ]

    def stop(self) -> Data:
        result = _rocos_post(self.base_url, "/api/script/stop", {})
        if result.get("success"):
            data = result.get("data", {})
            text = f"✅ 脚本已停止 → 状态: {data.get('state', 'STOPPED')}"
        else:
            text = f"⚠️ {result.get('message', '停止失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class GetScriptStatus(Component):
    """查询 Lua 脚本状态"""

    display_name: str = "查询脚本状态"
    description: str = (
        "查询当前 ROCOS 机器人上 Lua 脚本的执行状态。返回 script_id、状态 (LOADED/RUNNING/PAUSED/COMPLETED/FAILED/STOPPED)、"
        "当前文件名、行号、错误信息和断点列表。"
        "Use this to monitor script execution progress."
    )
    icon: str = "Eye"
    name: str = "get_script_status"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Script Status", name="script_status", method="fetch_status"),
    ]

    def fetch_status(self) -> Data:
        result = _rocos_get(self.base_url, "/api/script/status")
        if result.get("success"):
            data = result.get("data", {})
            text = (
                f"脚本 ID: {data.get('script_id', 'N/A')}\n"
                f"状态: {data.get('state', 'UNKNOWN')}\n"
                f"文件: {data.get('filename', '')} 行: {data.get('line', 0)}"
            )
            if data.get("error"):
                text += f"\n❌ 错误: {data['error']}"
            if data.get("breakpoints"):
                text += f"\n断点: {data['breakpoints']}"
        else:
            text = f"❌ {result.get('message', '查询失败')}"
        self.status = f"脚本状态: {result.get('data', {}).get('state', 'N/A')}"
        return Data(text=text, data=result)
