#!/bin/bash
set -e

# 获取脚本所在目录（无论从哪里执行都正确）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置资源路径（用于加载 policy.pt, gait.txt, trajectory 等）
export RESOURCE_PATH="$SCRIPT_DIR/share/RobotSystem/system"

# 可选：输出信息用于调试
echo "[RobotSystem] RESOURCE_PATH = $RESOURCE_PATH"
echo "[RobotSystem] Executable     = $SCRIPT_DIR/bin/RobotSystem"

# 启动程序
"$SCRIPT_DIR/bin/RobotSystem" "$@"
