# ROCOS Robot Control — Langflow 自定义 Tool 组件

Langflow 自定义组件集，封装了 ROCOS-App 的 HTTP REST API，使 LLM Agent 可以直接通过自然语言控制机器人。

## 组件列表 (35 个 Tool 组件)

| 组件名 | 类别 | 说明 |
|--------|------|------|
| `RocosConfig` | 配置 | ROCOS API 基础 URL 配置 |
| `GetRobotState` | 状态 | 获取机器人完整状态（关节、位姿、FSM、硬件） |
| `GetRobotInfo` | 状态 | 获取关节参数信息（传动比、编码器等） |
| `GetRobotEnabled` | 状态 | 查询机器人使能状态 |
| `GetRobotURDF` | 状态 | 获取机器人 URDF 模型 XML |
| `GetMotionStatus` | 状态 | 查询运动任务/FSM 状态 |
| `GetScriptStatus` | 状态 | 查询 Lua 脚本执行状态 |
| `EnableRobot` | 控制 | 全局使能机器人 |
| `DisableRobot` | 控制 | 全局禁用机器人 |
| `SetWorkMode` | 控制 | 设置工作模式 (position/阻抗/导纳) |
| `MoveJ` | 运动 | 关节空间运动 (MoveJ) |
| `MoveL` | 运动 | 笛卡尔直线运动 (MoveL) |
| `MotionStop` | 运动 | 停止运动 |
| `MotionPause` | 运动 | 暂停运动 |
| `MotionResume` | 运动 | 继续运动 |
| `WaitMove` | 运动 | 等待当前运动完成 |
| `JogStart` | 点动 | 启动点动 (关节/笛卡尔/零空间，新版向量接口) |
| `JogStop` | 点动 | 停止点动 |
| `GetToolFrames` | 坐标系 | 获取所有工具坐标系名称 |
| `GetToolFrame` | 坐标系 | 按名称获取工具坐标系位姿 |
| `SetToolFrame` | 坐标系 | 设置/新增工具坐标系 (TCP) |
| `ActivateToolFrame` | 坐标系 | 激活指定工具坐标系 |
| `RemoveToolFrame` | 坐标系 | 删除指定工具坐标系 |
| `GetObjectFrames` | 坐标系 | 获取所有工件坐标系名称 |
| `GetObjectFrame` | 坐标系 | 按名称获取工件坐标系位姿 |
| `SetObjectFrame` | 坐标系 | 设置/新增工件坐标系 |
| `ActivateObjectFrame` | 坐标系 | 激活指定工件坐标系 |
| `RemoveObjectFrame` | 坐标系 | 删除指定工件坐标系 |
| `LoadFrames` | 坐标系 | 从 YAML 文件加载坐标系 |
| `SaveFrames` | 坐标系 | 将坐标系保存到 YAML 文件 |
| `UploadScript` | Lua | 上传并编译 Lua 脚本 |
| `RunScript` | Lua | 异步执行 Lua 脚本 |
| `PauseScript` | Lua | 暂停 Lua 脚本 |
| `ResumeScript` | Lua | 继续 Lua 脚本 |
| `StopScript` | Lua | 停止 Lua 脚本 |

## 快速开始

### 1. 安装 Langflow

```bash
# 使用 uv (推荐)
uv pip install langflow --system

# 或使用 pip
python3 -m pip install langflow
```

### 2. 启动 ROCOS 机器人控制器

```bash
cd /path/to/rocos-app/build
./bin/rocosAppMain --sim=true --http_port=8080
```

### 3. 启动 Langflow (加载自定义组件)

```bash
# 设置自定义组件路径
export LANGFLOW_COMPONENTS_PATH="/home/sun/Documents/GitHub/rocos-app/langflow_components"

# (可选) 设置默认 ROCOS API 地址
export ROCOS_BASE_URL="http://localhost:8080"

# 启动 Langflow
langflow run
```

或使用一键启动脚本：

```bash
cd /home/sun/Documents/GitHub/rocos-app
bash langflow_components/start.sh
```

### 4. 在 Langflow UI 中使用

1. 打开浏览器访问 `http://localhost:7860`
2. 创建新 Flow
3. 在组件面板中找到 **"Rocos 配置"**，拖入 Flow，设置 base_url
4. 添加 **Agent** 组件
5. 在 Agent 的 **Toolkits** 设置中，从 **ROCO** 类别下勾选需要的 Tool
6. 测试示例对话:
   - "查询机器人当前状态"
   - "用关节运动让机器人运动到 [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]"
   - "直线运动末端到 x=0.5, y=0, z=0.8 的位置"
   - "上传一个让机器人画正方形的 Lua 脚本并执行"

## 目录结构

```
langflow_components/
├── rocos/                        # 组件包 (Category)
│   ├── __init__.py               # 导出所有组件
│   ├── rocos_config.py           # 配置组件
│   ├── rocos_client.py           # 内部 HTTP 工具函数
│   ├── rocos_robot_state.py      # 状态查询
│   ├── rocos_robot_control.py    # 使能/禁用/工作模式
│   ├── rocos_motion.py           # MoveJ/MoveL/Stop/Pause/Resume/Status
│   ├── rocos_jogging.py          # 拖拽示教/点动
│   ├── rocos_frames.py           # 坐标系管理 (Tool/Object Frame)
│   └── rocos_script.py           # Lua 脚本管理
├── start.sh                      # 一键启动脚本
└── README.md                     # 本文件
```

## 自定义组件开发说明

### 组件基类

所有组件继承 `langflow.custom.Component`，声明式定义 inputs/outputs:

```python
from langflow.custom import Component
from langflow.io import Output, StrInput

class MyTool(Component):
    display_name: str = "我的工具"
    description: str = "Agent 用来决定何时调用此工具的描述"
    icon: str = "Wrench"
    name: str = "my_tool"

    inputs = [StrInput(name="param1", ...)]
    outputs = [Output(display_name="Result", name="result", method="do_work")]

    def do_work(self) -> Data:
        return Data(text="Done", data={...})
```

### 新增 ROCOS API 端点

1. 在对应的 `rocos_*.py` 文件中新增 Component 类
2. 在 `__init__.py` 中添加 import 和 `__all__` 导出
3. 重启 Langflow

### Agent 工具模式

- `description` 字段是关键 — Agent 根据它决定何时调用工具
- 描述中应包含触发条件词（如 "Use this tool when..."）
- 错误处理：返回 `success: false` 和 `code`，Agent 根据错误码判断恢复策略
