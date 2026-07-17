#!/usr/bin/env python3
"""
生成 ROCOS + DeepSeek Langflow 工作流 JSON
用法: python3 gen_flow.py > rocos_workflow.json
"""
import json, urllib.request, gzip, os, sys

BASE = os.environ.get("LANGFLOW_URL", "http://localhost:7860")

def req(method, path, body=None, token=None):
    url = f"{BASE}{path}"
    data = json.dumps(body).encode() if body else None
    r = urllib.request.Request(url, data=data, method=method)
    r.add_header("Content-Type", "application/json")
    r.add_header("Accept-Encoding", "identity")
    if token:
        r.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(r, timeout=30) as resp:
            raw = resp.read()
            if raw[:2] == b'\x1f\x8b':
                raw = gzip.decompress(raw)
            return json.loads(raw)
    except Exception as e:
        msg = str(e)
        if hasattr(e, 'read'):
            try:
                body = e.read()
                if isinstance(body, bytes) and body[:2] == b'\x1f\x8b':
                    body = gzip.decompress(body)
                msg = body.decode()[:500]
            except:
                pass
        print(f"API Error {method} {path}: {msg}", file=sys.stderr)
        return None

# Auth
auth = req("GET", "/api/v1/auto_login")
if not auth:
    print("Cannot connect to Langflow. Make sure it's running on", BASE, file=sys.stderr)
    sys.exit(1)
token = auth["access_token"]

# Get component templates
all_data = req("GET", "/api/v1/all", token=token)

# Extract complete node templates
def get_node(comp_data):
    """从 /api/v1/all 的组件数据中提取完整的 node 对象"""
    if isinstance(comp_data, dict):
        # 有些组件已经有 node 字段, 有些本身就是 node
        node = comp_data.get("node", comp_data)
        if isinstance(node, dict):
            return json.loads(json.dumps(node))  # deep copy
    return None

deepseek_node = get_node(all_data.get("deepseek", {}).get("DeepSeekModelComponent", {}))
agent_node = get_node(all_data.get("models_and_agents", {}).get("Agent", {}))
chat_input_node = get_node(all_data.get("input_output", {}).get("ChatInput", {}))
chat_output_node = get_node(all_data.get("input_output", {}).get("ChatOutput", {}))

# ROCOS components
rocos_comps = all_data.get("rocos", {})
rocos_nodes = {}
for name, cdata in rocos_comps.items():
    # name format: "ext:rocos:ComponentName@extra"
    short_name = name.split(":")[-1].split("@")[0]  # e.g., "GetRobotState"
    node = get_node(cdata)
    if node:
        rocos_nodes[short_name] = node

print(f"DeepSeek: {'OK' if deepseek_node else 'MISSING'}", file=sys.stderr)
print(f"Agent: {'OK' if agent_node else 'MISSING'}", file=sys.stderr)
print(f"ChatInput: {'OK' if chat_input_node else 'MISSING'}", file=sys.stderr)
print(f"ChatOutput: {'OK' if chat_output_node else 'MISSING'}", file=sys.stderr)
print(f"ROCOS tools: {list(rocos_nodes.keys())}", file=sys.stderr)

if not all([deepseek_node, agent_node, chat_input_node, chat_output_node]):
    print("ERROR: Missing required components", file=sys.stderr)
    sys.exit(1)

# --- 生成 Flow JSON ---
flow_id = "rocos-deepseek-flow"
nodes_list = []
edges_list = []

# 辅助: 创建 node 的 data 结构
def make_node_data(node_template, comp_type, comp_id):
    """构建完整的 node data 字典"""
    return {
        "id": comp_id,
        "type": comp_type,
        "node": node_template,
    }

# 组件 ID (使用短 ID 便于识别)
DS_ID    = "DeepSeekModel-wEo6Xds"
AGENT_ID = "Agent-jGzEhag"
CI_ID    = "ChatInput-Ds8Nci"
CO_ID    = "ChatOutput-Rfz4co"
ROCONF_ID = "RocosConfig-Kfscfg"

# --- 节点位置布局 ---
#      [ChatInput]    [RocosConfig]
#           |               |
#        [Agent]  ← [DeepSeek]
#           |
#      [ChatOutput]
#  (ROCOS tools 通过 Agent 的 tools 字段连接)

positions = {
    CI_ID:    {"x": 100,  "y": 100},
    ROCONF_ID: {"x": 100,  "y": 350},
    DS_ID:    {"x": 500,  "y": 250},
    AGENT_ID: {"x": 350,  "y": 400},
    CO_ID:    {"x": 350,  "y": 700},
}

# --- Node 1: ChatInput ---
ci_node = json.loads(json.dumps(chat_input_node))
# 设置默认 input
ci_node["template"]["input_value"]["value"] = ""
ci_node["template"]["sender"]["value"] = "User"
ci_node["template"]["sender_name"]["value"] = "User"
ci_node["frozen"] = False
ci_node["minimized"] = False
nodes_list.append({
    "id": CI_ID,
    "data": make_node_data(ci_node, "ChatInput", CI_ID),
    "position": positions[CI_ID],
})

# --- Node 2: RocosConfig ---
if "RocosConfig" in rocos_nodes:
    rc_node = json.loads(json.dumps(rocos_nodes["RocosConfig"]))
    rc_node["template"]["base_url"]["value"] = "http://localhost:8080"
    rc_node["frozen"] = False
    rc_node["minimized"] = False
    nodes_list.append({
        "id": ROCONF_ID,
        "data": make_node_data(rc_node, "RocosConfig", ROCONF_ID),
        "position": positions[ROCONF_ID],
    })

# --- Node 3: DeepSeek ---
ds_node = json.loads(json.dumps(deepseek_node))
# 配置 DeepSeek
ds_node["template"]["model_name"]["value"] = "deepseek-chat"
ds_node["template"]["temperature"]["value"] = 0.7
ds_node["template"]["max_tokens"]["value"] = 4096
# api_base 留空则使用默认 DeepSeek API; 要中转站则填入中转站地址
ds_node["template"]["api_base"]["value"] = ""  # 留空=默认 https://api.deepseek.com/v1
# api_key 留空则从环境变量 DEEPSEEK_API_KEY 获取
ds_node["template"]["api_key"]["value"] = "DEEPSEEK_API_KEY"
ds_node["frozen"] = False
ds_node["minimized"] = False
ds_node["edited"] = True
nodes_list.append({
    "id": DS_ID,
    "data": make_node_data(ds_node, "DeepSeekModelComponent", DS_ID),
    "position": positions[DS_ID],
})

# --- Node 4: Agent ---
ag_node = json.loads(json.dumps(agent_node))
ag_node["template"]["system_prompt"]["value"] = (
    "你是 ROCOS 机器人控制助手。你可以:\n"
    "1. 查询机器人状态 (GetRobotState, GetRobotEnabled)\n"
    "2. 使能/禁用机器人 (EnableRobot, DisableRobot)\n"
    "3. 执行运动指令 (MoveJ, MoveL, MotionStop 等)\n"
    "4. 管理坐标系 (GetToolFrames, SetToolFrame 等)\n"
    "5. 执行 Lua 脚本\n\n"
    "在发送运动指令前，请先确认机器人已使能。"
    "机器人操作可能危险，执行前向用户确认关键动作。"
    "Always respond in Chinese (简体中文)."
)
ag_node["template"]["add_current_date_tool"]["value"] = False
ag_node["template"]["add_calculator_tool"]["value"] = False
ag_node["frozen"] = False
ag_node["minimized"] = False
nodes_list.append({
    "id": AGENT_ID,
    "data": make_node_data(ag_node, "Agent", AGENT_ID),
    "position": positions[AGENT_ID],
})

# --- Node 5: ChatOutput ---
co_node = json.loads(json.dumps(chat_output_node))
co_node["frozen"] = False
co_node["minimized"] = False
nodes_list.append({
    "id": CO_ID,
    "data": make_node_data(co_node, "ChatOutput", CO_ID),
    "position": positions[CO_ID],
})

# --- ROCOS Tool Nodes ---
roc_tool_ids = []
tool_y = 400
for i, (short_name, roc_node_tpl) in enumerate(rocos_nodes.items()):
    if short_name == "RocosConfig":
        continue  # already added above
    tool_id = f"Rocos{short_name}-LDq9r{i}"
    node = json.loads(json.dumps(roc_node_tpl))
    node["frozen"] = False
    node["minimized"] = False
    nodes_list.append({
        "id": tool_id,
        "data": make_node_data(node, short_name, tool_id),
        "position": {"x": 650, "y": tool_y},
    })
    roc_tool_ids.append(tool_id)
    tool_y += 80

# --- Edges ---
def make_edge(source_id, source_name, source_type, target_id, target_field, target_input_types):
    source_handle = json.dumps({
        "dataType": source_type,
        "id": source_id,
        "name": source_name,
        "output_types": ["Message"] if source_type == "ChatInput" else
                        ["LanguageModel"] if "Model" in source_type else
                        ["Tool"] if source_name == "component_as_tool" else
                        ["Message"]
    })
    target_handle = json.dumps({
        "fieldName": target_field,
        "id": target_id,
        "inputTypes": target_input_types,
        "type": "other" if target_field not in ("input_value", "model") else "str"
    })
    edge_id = f"xy-edge__{source_id}{source_handle}-{target_id}{target_handle}"
    return {
        "id": edge_id,
        "source": source_id,
        "sourceHandle": source_handle,
        "target": target_id,
        "targetHandle": target_handle,
        "data": {
            "sourceHandle": json.loads(source_handle),
            "targetHandle": json.loads(target_handle),
        }
    }

# ChatInput -> Agent (input_value)
edges_list.append(make_edge(CI_ID, "message", "ChatInput", AGENT_ID, "input_value", ["Message"]))

# DeepSeek Model: Langflow 1.10 中模型通过 Agent 内部下拉框选择，不需要连线
# 用户在 Flow 中点击 Agent → Model 字段 → 选择 DeepSeek 即可

# Agent -> ChatOutput
edges_list.append(make_edge(AGENT_ID, "response", "Agent", CO_ID, "input_value",
                            ["Data", "JSON", "DataFrame", "Table", "Message"]))

# Each ROCOS tool -> Agent (tools)
for tool_id in roc_tool_ids:
    edges_list.append(make_edge(tool_id, "component_as_tool", "Tool",
                                AGENT_ID, "tools", ["Tool"]))

# RocosConfig 的输出 (config) 暂时不连 — 用户需在 flow 中手动连或用环境变量
# RocosConfig 中的 base_url 可通过 Agent 的 context 或环境变量获取

# --- 构建最终 Flow ---
flow = {
    "name": "ROCOS 机器人控制",
    "description": "用 DeepSeek 驱动 ROCOS 机器人控制。包含状态查询、运动控制、坐标系管理、Lua 脚本等功能。",
    "data": {
        "nodes": nodes_list,
        "edges": edges_list,
    },
    "is_component": False,
    "folder_id": None,
    "tags": ["robotics", "rocos", "deepseek"],
}

print(json.dumps(flow, indent=2, ensure_ascii=False))
