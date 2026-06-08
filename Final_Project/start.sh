#!/bin/bash
# ============================================================
# 智能仓储机器人数字孪生系统 - 一键启动脚本
# 用法: bash start.sh
# ============================================================
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}============================================================${NC}"
echo -e "${CYAN}     🏭 智能仓储机器人数字孪生系统 - 一键启动${NC}"
echo -e "${CYAN}============================================================${NC}"

# ---- 获取脚本所在目录（Final_Project 目录） ----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"   # ROS2_Demo 目录
PKG_NAME="warehouse_digital_twin"

# ---- 自动创建 src 软链接 ----
if [ ! -L "$WS_DIR/src/$PKG_NAME" ]; then
    echo -e "${YELLOW}[1/5] 创建工作空间软链接...${NC}"
    mkdir -p "$WS_DIR/src"
    ln -sf "$SCRIPT_DIR" "$WS_DIR/src/$PKG_NAME"
    echo -e "${GREEN}  已链接: src/$PKG_NAME -> Final_Project${NC}"
else
    echo -e "${GREEN}[1/5] 工作空间软链接已存在${NC}"
fi

# ---- 检查并安装 Flask ----
echo -e "${YELLOW}[2/5] 检查依赖...${NC}"
if ! python3 -c "import flask" 2>/dev/null; then
    echo -e "${YELLOW}  Flask 未安装，正在安装...${NC}"
    pip install flask --user -q
    echo -e "${GREEN}  Flask 安装完成${NC}"
else
    echo -e "${GREEN}  Flask 已安装${NC}"
fi

# ---- 加载 ROS2 环境 ----
echo -e "${YELLOW}[3/5] 加载 ROS2 环境...${NC}"
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
else
    echo -e "${RED}[错误] 未找到 ROS2 安装！请先安装 ROS2${NC}"
    exit 1
fi
echo -e "${GREEN}  ROS2 环境已加载: $ROS_DISTRO${NC}"

# ---- 编译 ----
echo -e "${YELLOW}[4/5] 编译项目...${NC}"
cd "$WS_DIR"
colcon build --packages-select "$PKG_NAME" --symlink-install
source install/setup.bash
echo -e "${GREEN}  编译完成${NC}"

# ---- 获取服务器 IP ----
SERVER_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
[ -z "$SERVER_IP" ] && SERVER_IP=$(ip addr show 2>/dev/null | grep 'inet ' | grep -v 127.0.0.1 | awk '{print $2}' | cut -d/ -f1 | head -1)

# ---- 启动 ----
echo -e "${YELLOW}[5/5] 启动系统...${NC}"
echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  系统正在运行！${NC}"
echo -e "${GREEN}  🌐 浏览器打开: http://${SERVER_IP}:5000${NC}"
echo -e "${GREEN}  🛑 按 Ctrl+C 停止${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""

ros2 launch "$PKG_NAME" warehouse_twin_web.launch.py
