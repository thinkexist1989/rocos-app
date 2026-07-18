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
from .get_robot_state import GetRobotState
from .get_robot_info import GetRobotInfo
from .get_robot_enabled import GetRobotEnabled
from .get_robot_urdf import GetRobotURDF
from .enable_robot import EnableRobot
from .disable_robot import DisableRobot
from .set_work_mode import SetWorkMode
from .move_j import MoveJ
from .move_l import MoveL
from .motion_stop import MotionStop
from .motion_pause import MotionPause
from .motion_resume import MotionResume
from .get_motion_status import GetMotionStatus
from .wait_move import WaitMove
from .jog_start import JogStart
from .jog_stop import JogStop
from .get_tool_frames import GetToolFrames
from .get_tool_frame import GetToolFrame
from .set_tool_frame import SetToolFrame
from .activate_tool_frame import ActivateToolFrame
from .remove_tool_frame import RemoveToolFrame
from .get_object_frames import GetObjectFrames
from .get_object_frame import GetObjectFrame
from .set_object_frame import SetObjectFrame
from .activate_object_frame import ActivateObjectFrame
from .remove_object_frame import RemoveObjectFrame
from .load_frames import LoadFrames
from .save_frames import SaveFrames
from .upload_script import UploadScript
from .run_script import RunScript
from .pause_script import PauseScript
from .resume_script import ResumeScript
from .stop_script import StopScript
from .get_script_status import GetScriptStatus
from .rocos_agent_prompt import RocosAgentPrompt
from .rocos_error_recovery import RocosErrorRecovery
from .rocos_safe_movel import RocosSafeMoveL
from .reset_robot_fault import ResetRobotFault
from .rocos_safe_executor import RocosSafeExecutor
from .rocos_smart_executor import RocosSmartExecutor
from .rocos_semantic_cache import RocosSemanticCache
from .rocos_memory import RocosMemory
from .rocos_reflect import RocosReflect
from .rocos_knowledge import RocosKnowledge

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
    "RocosSmartExecutor",
    # Memory & Evolution
    "RocosSemanticCache", "RocosMemory", "RocosReflect", "RocosKnowledge",
]
