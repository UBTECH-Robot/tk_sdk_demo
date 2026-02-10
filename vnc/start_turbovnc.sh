#!/bin/bash

###############################################################################
# TurboVNC + noVNC 启动脚本
# 用于快速启动虚拟桌面并通过浏览器 noVNC 访问
# 支持 Ctrl+C 优雅关闭所有服务
###############################################################################

# 注意：不使用 set -e，因为 cleanup 函数需要在有错误时继续执行
# set -e 会导致脚本在错误时立即退出，跳过清理代码

# 配置
DISPLAY_NUM=":100"
VNC_PORT=$((5900 + ${DISPLAY_NUM#:}))
NOVNC_PORT="8888"
GEOMETRY="1920x1080"
DEPTH="24"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

# PID 记录变量
TURBOVNC_PID=""
NOVNC_PID=""

###############################################################################
# 清理函数 - 在脚本退出时执行
###############################################################################
cleanup() {
    echo ""
    echo -e "${YELLOW}==========================================${NC}"
    echo -e "${YELLOW}收到退出信号，正在清理服务...${NC}"
    echo -e "${YELLOW}==========================================${NC}"
    
    # 标志：是否发生了任何错误
    CLEANUP_ERROR=0
    
    # 1. 关闭 noVNC（优先关闭，因为它是 WebSocket 代理）
    if [ -n "$NOVNC_PID" ] && kill -0 "$NOVNC_PID" 2>/dev/null; then
        echo -e "${CYAN}[1/2] 关闭 noVNC (PID: $NOVNC_PID)...${NC}"
        if kill -TERM "$NOVNC_PID" 2>/dev/null; then
            # 等待进程优雅退出
            for i in {1..10}; do
                if ! kill -0 "$NOVNC_PID" 2>/dev/null; then
                    echo -e "${GREEN}  ✓ noVNC 已停止${NC}"
                    break
                fi
                if [ $i -eq 10 ]; then
                    echo -e "${YELLOW}  ⚠ noVNC 未立即退出，强制杀死...${NC}"
                    kill -KILL "$NOVNC_PID" 2>/dev/null || true
                fi
                sleep 0.3
            done
        fi
    fi
    
    # 2. 关闭 TurboVNC（使用官方方法）
    if [ -n "$DISPLAY_NUM" ]; then
        echo -e "${CYAN}[2/2] 关闭 TurboVNC ${DISPLAY_NUM}...${NC}"
        if [ -f /opt/TurboVNC/bin/vncserver ]; then
            if /opt/TurboVNC/bin/vncserver -kill "$DISPLAY_NUM" 2>/dev/null; then
                echo -e "${GREEN}  ✓ TurboVNC 已停止${NC}"
            else
                echo -e "${YELLOW}  ⚠ TurboVNC 关闭失败，尝试强制杀死...${NC}"
                pkill -f "vncserver.*${DISPLAY_NUM}" 2>/dev/null || true
                CLEANUP_ERROR=1
            fi
        fi
    fi
    
    # 3. 额外安全：检查是否还有遗留的进程
    echo -e "${CYAN}[3/3] 检查遗留进程...${NC}"
    
    # 检查 Xvfb
    if pgrep -f "Xvfb.*${DISPLAY_NUM}" &>/dev/null; then
        echo -e "${YELLOW}  发现遗留的 Xvfb 进程，清理...${NC}"
        if pkill -f "Xvfb.*${DISPLAY_NUM}" 2>/dev/null; then
            echo -e "${GREEN}    ✓ Xvfb 已清理${NC}"
        fi
    fi
    
    # 检查 gnome-session（无需权限验证，只是清理可访问的进程）
    GNOME_PIDS=$(pgrep -f "gnome-session.*${DISPLAY_NUM}" 2>/dev/null || true)
    if [ -n "$GNOME_PIDS" ]; then
        echo -e "${YELLOW}  发现遗留的 GNOME 进程...${NC}"
        # 尝试以用户身份杀死这些进程
        for PID in $GNOME_PIDS; do
            if kill -TERM "$PID" 2>/dev/null; then
                echo -e "${GREEN}    ✓ 已清理 PID $PID${NC}"
            fi
        done
    fi
    
    # 最终验证
    sleep 1
    if lsof -i :"$VNC_PORT" &>/dev/null; then
        echo -e "${RED}  ✗ VNC 端口 $VNC_PORT 仍被占用${NC}"
        CLEANUP_ERROR=1
    else
        echo -e "${GREEN}  ✓ VNC 端口已释放${NC}"
    fi
    
    if lsof -i :"$NOVNC_PORT" &>/dev/null; then
        echo -e "${RED}  ✗ noVNC 端口 $NOVNC_PORT 仍被占用${NC}"
        CLEANUP_ERROR=1
    else
        echo -e "${GREEN}  ✓ noVNC 端口已释放${NC}"
    fi
    
    echo ""
    if [ $CLEANUP_ERROR -eq 0 ]; then
        echo -e "${GREEN}✓ 所有服务已正常停止${NC}"
    else
        echo -e "${YELLOW}⚠ 部分服务停止时出现问题，请手动检查${NC}"
    fi
    
    echo -e "${YELLOW}========================================${NC}"
}

# 捕获信号并执行清理函数
trap cleanup SIGINT SIGTERM EXIT

echo -e "${GREEN}=== TurboVNC + noVNC 启动脚本 ===${NC}"
echo "显示号: $DISPLAY_NUM"
echo "VNC 端口: $VNC_PORT"
echo "noVNC 端口: $NOVNC_PORT"
echo ""
echo -e "${CYAN}提示: 按 Ctrl+C 可安全停止所有服务${NC}"
echo ""

# Step 1: 检查依赖
echo -e "${YELLOW}[1/4] 检查依赖...${NC}"
if [ ! -f /opt/TurboVNC/bin/vncserver ]; then
    echo -e "${RED}错误: TurboVNC 未安装${NC}"
    exit 1
fi
echo -e "${GREEN}✓ TurboVNC 已安装${NC}"

# Step 2: 准备 xstartup 脚本
echo -e "${YELLOW}[2/4] 准备 xstartup 脚本...${NC}"
mkdir -p "$HOME/.vnc"

if [ ! -f "$HOME/.vnc/xstartup" ] && [ -f "$SCRIPT_DIR/xstartup" ]; then
    echo "复制 xstartup 脚本..."
    cp "$SCRIPT_DIR/xstartup" "$HOME/.vnc/xstartup"
    chmod +x "$HOME/.vnc/xstartup"
    echo -e "${GREEN}✓ xstartup 已准备${NC}"
elif [ -f "$HOME/.vnc/xstartup" ]; then
    echo -e "${GREEN}✓ xstartup 已存在${NC}"
else
    echo -e "${YELLOW}⚠ 警告: 找不到 xstartup 脚本${NC}"
fi

# Step 3: 杀死旧的 VNC 进程
echo -e "${YELLOW}[3/4] 清理旧进程...${NC}"
pkill -f "vncserver.*${DISPLAY_NUM}" || true
sleep 1

# Step 4: 启动 TurboVNC
echo -e "${YELLOW}[4/4] 启动虚拟桌面...${NC}"
echo "TurboVNC 命令:"
echo "/opt/TurboVNC/bin/vncserver $DISPLAY_NUM -xstartup $HOME/.vnc/xstartup -geometry $GEOMETRY -depth $DEPTH -vgl"
echo ""

# 启动 TurboVNC（后台启动并记录 PID）
/opt/TurboVNC/bin/vncserver "$DISPLAY_NUM" \
    -xstartup "$HOME/.vnc/xstartup" \
    -geometry "$GEOMETRY" \
    -depth "$DEPTH" \
    -vgl &

TURBOVNC_PID=$!
echo -e "${GREEN}✓ TurboVNC 已启动 (监视 PID: $TURBOVNC_PID)${NC}"

# 等待 VNC 服务启动
echo "等待 VNC 服务启动..."
sleep 3

# 验证 VNC 端口是否监听
echo "验证 VNC 端口..."
if netstat -tulnp 2>/dev/null | grep -q ":$VNC_PORT" || lsof -i :$VNC_PORT 2>/dev/null; then
    echo -e "${GREEN}✓ VNC 在端口 $VNC_PORT 监听${NC}"
else
    echo -e "${YELLOW}⚠ 警告: 无法验证 VNC 端口状态${NC}"
    echo "查看日志: tail -50 ~/.vnc/xstartup.log"
fi

# Step 5: 启动 noVNC
echo ""
echo -e "${YELLOW}启动 noVNC 代理...${NC}"

if [ ! -d "$SCRIPT_DIR/pkg/noVNC" ]; then
    echo -e "${RED}错误: noVNC 目录不存在: $SCRIPT_DIR/pkg/noVNC${NC}"
    echo "VNC 已启动，但 noVNC 无法启动"
    echo "VNC 连接信息:"
    echo "  vncviewer localhost:$VNC_PORT"
    # 等待用户中断，清理函数会在 trap 中执行
    wait
    exit 1
fi

echo "noVNC 命令:"
echo "$SCRIPT_DIR/pkg/noVNC/utils/novnc_proxy \\"
echo "    --vnc localhost:$VNC_PORT \\"
echo "    --listen 0.0.0.0:$NOVNC_PORT \\"
echo "    --file-only \\"
echo "    --heartbeat 10"
echo ""

# 后台启动 noVNC 并记录 PID
"$SCRIPT_DIR/pkg/noVNC/utils/novnc_proxy" \
    --vnc "localhost:$VNC_PORT" \
    --listen "0.0.0.0:$NOVNC_PORT" \
    --file-only \
    --heartbeat 10 2>&1 &

NOVNC_PID=$!
echo -e "${GREEN}✓ noVNC 已启动 (PID: $NOVNC_PID)${NC}"

# 等待 noVNC 启动
sleep 2

# 最终显示连接信息
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✓ 虚拟桌面启动完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}连接信息:${NC}"
echo ""
echo "方式 1: VNC 直接连接"
echo "  -命令: vncviewer localhost:$VNC_PORT"
echo "  -或使用 VNC 客户端连接到 localhost:$VNC_PORT"
echo ""
echo "方式 2: 网页浏览器 (noVNC)"
echo "  -本地: http://localhost:$NOVNC_PORT/vnc.html"
HOSTNAME_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "SERVER_IP")
echo "  -远程: http://$HOSTNAME_IP:$NOVNC_PORT/vnc.html"
echo ""
echo "方式 3: SSH 隧道 + VNC"
echo "  -远程服务器上运行此脚本"
echo "  -本地建立隧道: ssh -L $VNC_PORT:localhost:$VNC_PORT user@server"
echo "  -本地连接: vncviewer localhost:$VNC_PORT"
echo ""

echo -e "${YELLOW}日志文件:${NC}"
echo "  xstartup 日志: tail -f ~/.vnc/xstartup.log"
echo "  VNC 日志: tail -f ~/.vnc/${DISPLAY_NUM##:}.log"
echo "  noVNC 输出: 直接显示在上面"
echo ""

echo -e "${CYAN}进程信息:${NC}"
echo "  TurboVNC PID: $TURBOVNC_PID"
echo "  noVNC PID: $NOVNC_PID"
echo ""

echo -e "${YELLOW}停止服务:${NC}"
echo "  按 Ctrl+C 可安全停止所有服务"
echo ""
echo -e "${CYAN}==========================================${NC}"
echo -e "${CYAN}服务运行中... (Ctrl+C 停止)${NC}"
echo -e "${CYAN}==========================================${NC}"
echo ""

# 持续运行脚本，直到接收到 Ctrl+C
# 定期检查进程是否仍在运行
while true; do
    # 检查 TurboVNC 进程状态
    if ! kill -0 "$TURBOVNC_PID" 2>/dev/null; then
        echo -e "${RED}错误: TurboVNC 进程已退出${NC}"
        echo "请检查日志: tail ~/.vnc/xstartup.log"
        sleep 5
        # 等待用户中断，cleanup 函数会执行
        continue
    fi
    
    # 检查 noVNC 进程状态
    if ! kill -0 "$NOVNC_PID" 2>/dev/null; then
        echo -e "${RED}错误: noVNC 进程已退出${NC}"
        sleep 5
        # 等待用户中断，cleanup 函数会执行
        continue
    fi
    
    # 两个进程都在运行，等待 1 秒后再检查
    sleep 1
done
