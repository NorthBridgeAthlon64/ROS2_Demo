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
VENV_DIR="$SCRIPT_DIR/venv"

# ---- 自动创建 src 软链接 ----
echo -e "${YELLOW}[1/6] 创建工作空间软链接...${NC}"
if [ ! -L "$WS_DIR/src/$PKG_NAME" ]; then
    mkdir -p "$WS_DIR/src"
    ln -sf "$SCRIPT_DIR" "$WS_DIR/src/$PKG_NAME"
    echo -e "${GREEN}  已链接: src/$PKG_NAME -> Final_Project${NC}"
else
    echo -e "${GREEN}  工作空间软链接已存在${NC}"
fi

# ---- 创建或激活 Python 虚拟环境 ----
echo -e "${YELLOW}[2/6] 准备 Python 虚拟环境...${NC}"
# --system-site-packages 让 venv 能访问系统 ROS2 包（rclpy 等）
if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo -e "${YELLOW}  正在创建虚拟环境...${NC}"
    rm -rf "$VENV_DIR" 2>/dev/null  # 清理可能损坏的旧目录
    python3 -m venv --system-site-packages "$VENV_DIR"
    echo -e "${GREEN}  虚拟环境已创建${NC}"
else
    echo -e "${GREEN}  虚拟环境已就绪${NC}"
fi
source "$VENV_DIR/bin/activate"
echo -e "${GREEN}  虚拟环境已激活${NC}"

# ---- 安装 Python 依赖（仅在虚拟环境内） ----
echo -e "${YELLOW}[3/6] 检查 Python 依赖...${NC}"
if ! python -c "import flask" 2>/dev/null; then
    echo -e "${YELLOW}  Flask 未安装，正在安装到虚拟环境...${NC}"
    pip install flask -q
    echo -e "${GREEN}  Flask 安装完成${NC}"
else
    echo -e "${GREEN}  Flask 已安装${NC}"
fi

# ---- 加载 ROS2 环境 ----
echo -e "${YELLOW}[4/6] 加载 ROS2 环境...${NC}"
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
echo -e "${YELLOW}[5/6] 编译项目...${NC}"
cd "$WS_DIR"
colcon build --packages-select "$PKG_NAME" --symlink-install
source install/setup.bash
echo -e "${GREEN}  编译完成${NC}"

# ---- 获取服务器 IP ----
SERVER_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
[ -z "$SERVER_IP" ] && SERVER_IP=$(ip addr show 2>/dev/null | grep 'inet ' | grep -v 127.0.0.1 | awk '{print $2}' | cut -d/ -f1 | head -1)

# ---- 启动 ----
echo -e "${YELLOW}[6/6] 启动系统...${NC}"
echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  系统正在运行！${NC}"
echo -e "${GREEN}  🌐 浏览器打开: http://${SERVER_IP}:5000${NC}"
echo -e "${GREEN}  🛑 按 Ctrl+C 停止${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""

ros2 launch "$PKG_NAME" warehouse_twin_web.launch.py
