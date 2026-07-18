"""
ROCOS Robot Control - Langflow 自定义 Tool 组件包

封装 ROCOS-App HTTP REST API，使 Langflow 的 LLM Agent 可以直接控制机器人。
支持的 API 类别:
  - 机器人状态查询 (/api/robot/state, /api/robot/info, /api/robot/enabled)
  - 机器人控制 (/api/robot/enable, /api/robot/disable, /api/robot/workmode)
  - 运动控制   (/api/robot/movej, /api/robot/movel, /api/robot/stop, ...)
  - 点动控制   (/api/robot/jog/joint, /api/robot/jog/cartesian, ...)
  - 坐标系管理 (/api/robot/tool_frame/, /api/robot/object_frame/)
  - Lua 脚本   (/api/script/upload, /api/script/run, /api/script/status, ...)

使用方法:
  1. 设置环境变量 LANGFLOW_COMPONENTS_PATH 指向 langflow_components/ 目录
  2. 在 .env 或工作流中设置 ROCOS_BASE_URL（默认 http://localhost:8080）
  3. 在 Agent 的 Toolkits 中选择需要的 ROCOS 工具
"""

from .rocos_config import RocosConfig
from .rocos_robot_state import GetRobotState, GetRobotInfo, GetRobotEnabled, GetRobotURDF
from .rocos_robot_control import EnableRobot, DisableRobot, SetWorkMode
from .rocos_motion import MoveJ, MoveL, MotionStop, MotionPause, MotionResume, GetMotionStatus, WaitMove
from .rocos_jogging import JogStart, JogStop
from .rocos_frames import (
    GetToolFrames, GetToolFrame, SetToolFrame, ActivateToolFrame, RemoveToolFrame,
    GetObjectFrames, GetObjectFrame, SetObjectFrame, ActivateObjectFrame, RemoveObjectFrame,
    LoadFrames, SaveFrames,
)
from .rocos_script import UploadScript, RunScript, PauseScript, ResumeScript, StopScript, GetScriptStatus
from .rocos_agent import RocosAgentPrompt, RocosErrorRecovery, RocosSafeMoveL, ResetRobotFault, RocosSafeExecutor
from .rocos_memory import RocosSemanticCache, RocosMemory, RocosReflect, RocosKnowledge

__all__ = [
    # Config
    "RocosConfig",
    # State
    "GetRobotState", "GetRobotInfo", "GetRobotEnabled", "GetRobotURDF",
    # Control
    "EnableRobot", "DisableRobot", "SetWorkMode",
    # Motion
    "MoveJ", "MoveL", "MotionStop", "MotionPause", "MotionResume", "GetMotionStatus", "WaitMove",
    # Jogging
    "JogStart", "JogStop",
    # Frames
    "GetToolFrames", "GetToolFrame", "SetToolFrame", "ActivateToolFrame", "RemoveToolFrame",
    "GetObjectFrames", "GetObjectFrame", "SetObjectFrame", "ActivateObjectFrame", "RemoveObjectFrame",
    "LoadFrames", "SaveFrames",
    # Script
    "UploadScript", "RunScript", "PauseScript", "ResumeScript", "StopScript", "GetScriptStatus",
    # Semantic layer (实战迭代)
    "RocosAgentPrompt", "RocosErrorRecovery", "RocosSafeMoveL", "ResetRobotFault", "RocosSafeExecutor",
    # Memory & Evolution
    "RocosSemanticCache", "RocosMemory", "RocosReflect", "RocosKnowledge",
]
