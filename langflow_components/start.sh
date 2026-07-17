#!/bin/bash
# ============================================================================
# ROCOS + Langflow 一键启动脚本
#
# 用法:
#   bash start.sh                            # 仅启动 Langflow
#   bash start.sh --rocos                    # 同时启动 Langflow 和 ROCOS 仿真
#   bash start.sh --rocos --rocos-port 9090   # 自定义 ROCOS 端口
#
# 环境变量:
#   LANGFLOW_COMPONENTS_PATH  - 自定义组件路径 (自动设置)
#   ROCOS_BASE_URL            - ROCOS API 地址 (默认 http://localhost:8080)
#   LANGFLOW_PORT             - Langflow 端口 (默认 7860)
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---- 默认配置 ----
ROCOS_PORT="${ROCOS_PORT:-8080}"
ROCOS_HOST="${ROCOS_HOST:-0.0.0.0}"
LANGFLOW_PORT="${LANGFLOW_PORT:-7860}"
ROCOS_BASE_URL="${ROCOS_BASE_URL:-http://localhost:${ROCOS_PORT}}"

# ---- 解析参数 ----
START_ROCOS=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --rocos)       START_ROCOS=true; shift ;;
        --rocos-port)  ROCOS_PORT="$2"; shift 2 ;;
        --rocos-host)  ROCOS_HOST="$2"; shift 2 ;;
        --langflow-port) LANGFLOW_PORT="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo "============================================"
echo "  ROCOS + Langflow 启动脚本"
echo "============================================"
echo "  项目目录:     ${PROJECT_DIR}"
echo "  ROCOS API:    ${ROCOS_BASE_URL}"
echo "  Langflow 端口: ${LANGFLOW_PORT}"
echo "  自定义组件:   ${SCRIPT_DIR}"
echo "============================================"

# ---- 1. 启动 ROCOS (可选) ----
if $START_ROCOS; then
    ROCOS_BIN="${PROJECT_DIR}/build/bin/rocosAppMain"
    if [ ! -f "$ROCOS_BIN" ]; then
        echo "[ERROR] ROCOS 可执行文件不存在: $ROCOS_BIN"
        echo "  请先构建项目: cd $PROJECT_DIR && cmake --build build"
        exit 1
    fi
    echo ""
    echo "[1/2] 启动 ROCOS (仿真模式)..."
    $ROCOS_BIN --sim=true --http_port="$ROCOS_PORT" --http_host="$ROCOS_HOST" &
    ROCOS_PID=$!
    echo "  ROCOS PID: $ROCOS_PID"
    sleep 2
    echo "  ROCOS 启动完成"
else
    echo ""
    echo "[INFO] 跳过 ROCOS 启动 (使用 --rocos 参数同时启动)"
fi

# ---- 2. 启动 Langflow ----
echo ""
echo "[2/2] 启动 Langflow..."

export LANGFLOW_COMPONENTS_PATH="$SCRIPT_DIR"
export ROCOS_BASE_URL="$ROCOS_BASE_URL"

echo "  LANGFLOW_COMPONENTS_PATH=$LANGFLOW_COMPONENTS_PATH"
echo "  ROCOS_BASE_URL=$ROCOS_BASE_URL"

# 清理函数
cleanup() {
    echo ""
    echo "正在关闭..."
    if [ -n "$ROCOS_PID" ]; then
        kill "$ROCOS_PID" 2>/dev/null && echo "  ROCOS 已停止 (PID $ROCOS_PID)"
    fi
    echo "  Langflow 已停止"
    exit 0
}
trap cleanup SIGINT SIGTERM

python3 -m langflow run --port "$LANGFLOW_PORT" --host "0.0.0.0"

cleanup
