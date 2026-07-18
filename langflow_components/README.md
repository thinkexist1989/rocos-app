# ROCOS Robot Control — Langflow 自定义 Tool 组件

Langflow 自定义组件集，封装 ROCOS-App HTTP REST API。**43 个组件，包含安全执行器、自进化记忆、语义缓存**。

## 架构

```
用户输入 "往右一点点"
  → RocosSemanticCache.query()       ← 语义缓存查询 (~0.5s)
  → 命中? → RocosSafeExecutor        ← 安全执行 + 自动记录
  → 未命中? → Agent LLM推理 → ...   ← ~8s
  → RocosSafeExecutor.execute()
    → 读状态 → 安全检查 → 执行 → WaitMove → 自动记录 → 缓存学习
```

## 组件列表 (44 个)

| 类别 | 组件 | 说明 |
|------|------|------|
| **配置** | `RocosConfig` | API 基础 URL 配置 |
| **状态查询** | `GetRobotState`, `GetRobotInfo`, `GetRobotEnabled`, `GetRobotURDF`, `GetMotionStatus`, `GetScriptStatus` | 机器人状态信息 |
| **基础控制** | `EnableRobot`, `DisableRobot`, `SetWorkMode` | 使能/禁用/工作模式 |
| **运动控制** | `MoveJ`, `MoveL`, `MotionStop`, `MotionPause`, `MotionResume`, `WaitMove` | 核心运动指令 |
| **点动** | `JogStart`, `JogStop` | 关节/笛卡尔/零空间点动 |
| **坐标系** | `GetToolFrames`, `GetToolFrame`, `SetToolFrame`, `ActivateToolFrame`, `RemoveToolFrame` | 工具坐标系 CRUD |
| **坐标系** | `GetObjectFrames`, `GetObjectFrame`, `SetObjectFrame`, `ActivateObjectFrame`, `RemoveObjectFrame` | 工件坐标系 CRUD |
| **坐标系** | `LoadFrames`, `SaveFrames` | YAML 批量导入/导出 |
| **Lua 脚本** | `UploadScript`, `RunScript`, `PauseScript`, `ResumeScript`, `StopScript` | 脚本管理 |
| **语义层** | `RocosAgentPrompt`, `RocosSafeMoveL`, `RocosSafeExecutor` | 安全执行 + 提示词 |
| **容错** | `RocosErrorRecovery`, `ResetRobotFault` | 错误恢复 + 复位 |
| **自进化** | `RocosSemanticCache`, `RocosMemory`, `RocosReflect`, `RocosKnowledge` | 缓存/记忆/自省/知识 |

## 快速开始

```bash
# 1. 启动 ROCOS
cd build/bin && ./rocosAppMain --http_port=8080

# 2. 启动 Langflow
export LANGFLOW_COMPONENTS_PATH="/path/to/langflow_components"
export DEEPSEEK_API_KEY="sk-xxx"
langflow run

# 3. 导入 Workflow
# 基础版: workflows/rocos_basic.json  (Agent + DeepSeek + ROCOS Tools)
```

## 目录结构

```
langflow_components/
├── .gitignore
├── README.md
├── start.sh                      # 一键启动
├── experiment.py                 # 自主试验脚本（开发工具）
├── gen_flow.py                   # Workflow 生成器
├── workflows/
│   └── rocos_basic.json          # 基础版 (Agent + DeepSeek + ROCOS Tools)
└── rocos/
    ├── __init__.py               # 43 组件导出
    ├── rocos_client.py           # 统一 HTTP 客户端
    ├── rocos_config.py           # 配置
    ├── rocos_robot_state.py      # 状态查询
    ├── rocos_robot_control.py    # 使能/禁用/模式
    ├── rocos_motion.py           # MoveJ/L/Stop/...
    ├── rocos_jogging.py          # 点动
    ├── rocos_frames.py           # 坐标系管理
    ├── rocos_script.py           # Lua 脚本
    ├── rocos_agent.py            # 语义层 + 安全执行器 + 容错
    └── rocos_memory.py           # 记忆 + 自省 + 缓存
```

## 安全模型

`RocosSafeExecutor` 是推荐的运动入口，自动完成：

```text
读状态 → 检查 STOPPED+enabled
       → 检查笛卡尔单步 ≤5cm
       → 检查关节单步 ≤0.3rad
       → 检查速度 ≤0.5
       → 执行 MoveL/MoveJ/Nullspace
       → WaitMove
       → 自动记录 Memory
       → 自动学习 SemanticCache
```

失败分层：
- 规划失败 (IK/限位) → 保持 STOPPED，调整参数重试
- 执行失败 (Fault) → ERROR_STATE → ResetRobotFault 复位
