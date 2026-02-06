#!/bin/bash

###############################################################################
# VNC Service Management Script (Xvfb + x11vnc + noVNC)
# Supports: start, stop, status commands
# Description: Manages Xvfb, x11vnc and noVNC services
###############################################################################

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running with sudo
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}错误: 此脚本需要sudo权限运行${NC}"
    echo "请使用: sudo $0 [start|stop|status]"
    exit 1
fi

# Get the actual user (not root)
ACTUAL_USER=${SUDO_USER:-$USER}
USER_HOME=$(eval echo ~$ACTUAL_USER)

# VNC display number (can be customized)
DISPLAY_NUM=0
VNC_DISPLAY=":${DISPLAY_NUM}"
VNC_PORT=$((5900 + DISPLAY_NUM))
NOVNC_PORT=8080

# Desktop environment selection (gnome or xfce)
DESKTOP_ENV="gnome"

# Function to show usage
show_usage() {
    echo "用法: sudo $0 {start|stop|status}"
    echo ""
    echo "命令:"
    echo "  start   - 启动 VNC 和 noVNC 服务"
    echo "  stop    - 停止 VNC 和 noVNC 服务"
    echo "  status  - 查看 VNC 和 noVNC 服务状态"
    echo ""
    exit 1
}

# Function to check if VNC is installed
check_vnc_installed() {
    if ! command -v Xvfb &> /dev/null; then
        echo -e "${RED}错误: Xvfb 未安装${NC}"
        echo "请先运行 install_vnc.sh 安装脚本"
        exit 1
    fi
    
    if ! command -v x11vnc &> /dev/null; then
        echo -e "${RED}错误: x11vnc 未安装${NC}"
        echo "请先运行 install_vnc.sh 安装脚本"
        exit 1
    fi
}

# Function to get server IP
get_server_ip() {
    hostname -I | awk '{print $1}'
}

# Function to check if virtual desktop session is running
# Checks if a real X11 session (with window manager) is active on the display
check_virtual_desktop_running() {
    # First, check if Xvfb is even running
    if ! pgrep -f "Xvfb ${VNC_DISPLAY}" > /dev/null 2>&1; then
        return 1  # Xvfb not running, so no virtual display at all
    fi
    
    # Try to query the root window - this will only work if a session is truly active
    if DISPLAY="${VNC_DISPLAY}" xwininfo -root > /dev/null 2>&1; then
        # Root window is accessible, check if there are actual windows managed
        if DISPLAY="${VNC_DISPLAY}" wmctrl -l > /dev/null 2>&1; then
            # wmctrl works, window manager is definitely running
            return 0
        fi
        
        # Even if wmctrl doesn't work, if xwininfo works and we have a process with this DISPLAY,
        # we need to be more careful - check if there's actually a session manager
        # Try to get the number of windows
        WINDOW_COUNT=$(DISPLAY="${VNC_DISPLAY}" wmctrl -l 2>/dev/null | wc -l)
        if [ "$WINDOW_COUNT" -gt 0 ]; then
            return 0
        fi
        
        # Last resort: check if we can access the display but this might just be Xvfb
        # without any real session. Look for actual session processes.
        if pgrep -f "DISPLAY=${VNC_DISPLAY}" | grep -qv "Xvfb\|x11vnc"; then
            return 0  # Found non-Xvfb/x11vnc processes on this display
        fi
    fi
    
    return 1  # Xvfb running but no real session
}

# Function to start VNC services
start_vnc() {
    echo -e "${GREEN}=== 启动 VNC 服务 (Xvfb + x11vnc + noVNC) ===${NC}"
    
    # Check if Xvfb is already running
    if pgrep -f "Xvfb ${VNC_DISPLAY}" > /dev/null; then
        echo -e "${YELLOW}Xvfb 虚拟显示服务已在运行中${NC}"
    else
        echo "启动 Xvfb 虚拟显示服务器..."
        
        # Set environment variables
        export DISPLAY="${VNC_DISPLAY}"
        export XDG_SESSION_TYPE=x11
        export GDK_BACKEND=x11
        
        # Start Xvfb in background
        Xvfb "${VNC_DISPLAY}" -ac -screen "0" "1920x1080x24" -dpi "96" \
            +extension "RANDR" +extension "GLX" +iglx +extension "MIT-SHM" \
            +render -nolisten "tcp" -noreset -shmem > /tmp/xvfb.log 2>&1 &
        
        XVFB_PID=$!
        echo "Xvfb PID: $XVFB_PID"
        
        # Wait for X socket
        echo "等待 X socket 就绪..."
        WAIT_COUNT=0
        while [ ! -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ] && [ $WAIT_COUNT -lt 30 ]; do
            sleep 1
            WAIT_COUNT=$((WAIT_COUNT + 1))
        done
        
        if [ -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ]; then
            echo -e "${GREEN}X socket 已就绪${NC}"
            echo -e "${GREEN}Xvfb 虚拟显示服务器已启动${NC}"
        else
            echo -e "${RED}X socket 启动超时${NC}"
            return 1
        fi
    fi
    
    # Give Xvfb a moment to stabilize
    sleep 2
    
    # Check if x11vnc is already running
    if pgrep -f "x11vnc.*display ${VNC_DISPLAY}" > /dev/null; then
        echo -e "${YELLOW}x11vnc 服务已在运行中${NC}"
    else
        echo "启动 x11vnc 服务..."
        
        # Start x11vnc
        x11vnc -display "${VNC_DISPLAY}" -shared -nomodtweak -forever \
            -capslock -repeat -xkb -xrandr "resize" \
            -rfbport ${VNC_PORT} -noshm > /tmp/x11vnc.log 2>&1 &
        
        X11VNC_PID=$!
        echo "x11vnc PID: $X11VNC_PID"
        sleep 2
        
        if pgrep -f "x11vnc.*display ${VNC_DISPLAY}" > /dev/null; then
            echo -e "${GREEN}x11vnc 服务已启动${NC}"
        else
            echo -e "${RED}x11vnc 服务启动失败，检查 /tmp/x11vnc.log${NC}"
            return 1
        fi
    fi
    
    # Check if noVNC is already running
    if pgrep -f "websockify.*${NOVNC_PORT}" > /dev/null; then
        echo -e "${YELLOW}noVNC 服务已在运行中${NC}"
    else
        echo "启动 noVNC 服务..."
        
        # Find noVNC path
        NOVNC_PATH="$SCRIPT_DIR/pkg/noVNC"
        
        if [ ! -d "$NOVNC_PATH" ]; then
            echo -e "${RED}错误: 找不到 noVNC 目录: $NOVNC_PATH${NC}"
            echo "请先运行 install_vnc.sh 安装脚本"
            return 1
        fi
        
        # Start noVNC
        cd "$NOVNC_PATH"
        ./utils/novnc_proxy --vnc localhost:${VNC_PORT} --listen ${NOVNC_PORT} \
            --heartbeat 10 > /tmp/novnc.log 2>&1 &
        
        NOVNC_PID=$!
        echo "noVNC PID: $NOVNC_PID"
        sleep 2
        
        if pgrep -f "websockify.*${NOVNC_PORT}" > /dev/null; then
            echo -e "${GREEN}noVNC 服务已启动${NC}"
        else
            echo -e "${RED}noVNC 服务启动失败，检查 /tmp/novnc.log${NC}"
            return 1
        fi
    fi
    
    # Check if desktop session is running (only vglrun-launched sessions)
    if check_virtual_desktop_running; then
        echo -e "${YELLOW}虚拟桌面会话已在运行中${NC}"
    else
        echo "启动虚拟桌面会话..."
        
        # Detect which desktop environment to use
        if dpkg -l 2>/dev/null | grep -q "^ii.*gnome-shell"; then
            DESKTOP_ENV="gnome"
            DESKTOP_CMD="gnome-session"
            echo "检测到 GNOME 桌面环境，将使用 GNOME"
        elif dpkg -l 2>/dev/null | grep -q "^ii.*xfce4"; then
            DESKTOP_ENV="xfce"
            DESKTOP_CMD="startxfce4"
            echo "检测到 XFCE 桌面环境，将使用 XFCE"
        else
            DESKTOP_ENV="gnome"
            DESKTOP_CMD="gnome-session"
            echo "未检测到桌面环境，尝试使用 GNOME"
        fi
        
        # Set VirtualGL environment if available
        if command -v vglrun &> /dev/null; then
            export VGL_DISPLAY="egl"
            export VGL_REFRESHRATE="60"
            echo -e "${GREEN}VirtualGL 已启用，使用 GPU 加速${NC}"
            
            # Start desktop session with VirtualGL
            DISPLAY="${VNC_DISPLAY}" vglrun +wm "$DESKTOP_CMD" > "/tmp/${DESKTOP_ENV}-session.log" 2>&1 &
        else
            echo -e "${YELLOW}VirtualGL 未安装，使用软件渲染${NC}"
            
            # Start desktop session without VirtualGL
            DISPLAY="${VNC_DISPLAY}" "$DESKTOP_CMD" > "/tmp/${DESKTOP_ENV}-session.log" 2>&1 &
        fi
        
        DESKTOP_PID=$!
        echo "桌面会话 ($DESKTOP_ENV) PID: $DESKTOP_PID"
        sleep 3
        
        if check_virtual_desktop_running; then
            echo -e "${GREEN}虚拟桌面会话已启动${NC}"
        else
            echo -e "${YELLOW}虚拟桌面会话启动可能需要更多时间，请检查 /tmp/${DESKTOP_ENV}-session.log${NC}"
        fi
    fi
    
    # Show connection information
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}VNC 服务启动成功！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    SERVER_IP=$(get_server_ip)
    echo -e "${BLUE}VNC 客户端连接:${NC}"
    echo "  地址: $SERVER_IP:${VNC_PORT}"
    echo "  或: $SERVER_IP${VNC_DISPLAY}"
    echo ""
    echo -e "${BLUE}浏览器访问 (noVNC):${NC}"
    echo "  地址: http://$SERVER_IP:${NOVNC_PORT}/vnc.html"
    echo ""
    echo -e "${BLUE}显示:${NC} ${VNC_DISPLAY}"
    echo -e "${BLUE}分辨率:${NC} 1920x1080"
    echo ""
    
    if command -v vglrun &> /dev/null; then
        echo -e "${GREEN}🎉 VirtualGL GPU 加速已启用${NC}"
        echo "OpenGL 应用将自动使用 GPU 硬件加速"
    fi
    echo ""
}

# Function to stop VNC services
stop_vnc() {
    echo -e "${GREEN}=== 停止 VNC 服务 ===${NC}"
    
    # Stop processes started by vglrun (virtual desktop sessions)
    echo "停止通过 vglrun 启动的虚拟桌面会话..."
    pkill -f "vglrun.*gnome-session" 2>/dev/null
    pkill -f "vglrun.*xfce" 2>/dev/null
    pkill -f "vglrun.*startxfce4" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}虚拟桌面会话已停止${NC}"
    else
        echo -e "${YELLOW}虚拟桌面会话未运行或已停止${NC}"
    fi
    
    # Stop noVNC
    echo "停止 noVNC 服务..."
    pkill -f "websockify.*${NOVNC_PORT}"
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}noVNC 服务已停止${NC}"
    else
        echo -e "${YELLOW}noVNC 服务未运行或已停止${NC}"
    fi
    
    # Stop x11vnc
    echo "停止 x11vnc 服务..."
    pkill -f "x11vnc.*display ${VNC_DISPLAY}"
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}x11vnc 服务已停止${NC}"
    else
        echo -e "${YELLOW}x11vnc 服务未运行或已停止${NC}"
    fi
    
    # Stop Xvfb
    echo "停止 Xvfb 虚拟显示服务器..."
    pkill -f "Xvfb ${VNC_DISPLAY}"
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Xvfb 虚拟显示服务器已停止${NC}"
    else
        echo -e "${YELLOW}Xvfb 虚拟显示服务器未运行或已停止${NC}"
    fi
    
    # Clean up lock files and temporary files
    echo "清理临时文件和锁文件..."
    rm -f "/tmp/.X11-unix/X${DISPLAY_NUM}" 2>/dev/null
    rm -f "/tmp/.X${DISPLAY_NUM}-lock" 2>/dev/null
    rm -f /tmp/xvfb.log /tmp/x11vnc.log /tmp/novnc.log /tmp/gnome-session.log /tmp/xfce-session.log 2>/dev/null
    echo "清理完成"
    
    echo ""
    echo -e "${GREEN}VNC 服务已完全停止${NC}"
    echo ""
}

# Function to show VNC status
status_vnc() {
    echo -e "${GREEN}=== VNC 服务状态 ===${NC}"
    echo ""
    
    SERVER_IP=$(get_server_ip)
    XVFB_RUNNING=false
    X11VNC_RUNNING=false
    NOVNC_RUNNING=false
    GNOME_RUNNING=false
    
    # Check Xvfb status
    echo -e "${BLUE}[Xvfb 虚拟显示服务器状态]${NC}"
    if pgrep -f "Xvfb ${VNC_DISPLAY}" > /dev/null; then
        echo -e "状态: ${GREEN}运行中${NC}"
        XVFB_RUNNING=true
        
        XVFB_PID=$(pgrep -f "Xvfb ${VNC_DISPLAY}")
        echo "进程 ID: $XVFB_PID"
        echo "显示编号: ${VNC_DISPLAY}"
        echo "分辨率: 1920x1080x24"
    else
        echo -e "状态: ${RED}未运行${NC}"
    fi
    
    echo ""
    
    # Check x11vnc status
    echo -e "${BLUE}[x11vnc 服务器状态]${NC}"
    if pgrep -f "x11vnc.*display ${VNC_DISPLAY}" > /dev/null; then
        echo -e "状态: ${GREEN}运行中${NC}"
        X11VNC_RUNNING=true
        
        X11VNC_PID=$(pgrep -f "x11vnc.*display ${VNC_DISPLAY}")
        echo "进程 ID: $X11VNC_PID"
        echo "监听端口: ${VNC_PORT}"
        echo "连接地址: $SERVER_IP:${VNC_PORT}"
    else
        echo -e "状态: ${RED}未运行${NC}"
    fi
    
    echo ""
    
    # Check noVNC status
    echo -e "${BLUE}[noVNC 服务状态]${NC}"
    if pgrep -f "websockify.*${NOVNC_PORT}" > /dev/null; then
        echo -e "状态: ${GREEN}运行中${NC}"
        NOVNC_RUNNING=true
        
        NOVNC_PID=$(pgrep -f "websockify.*${NOVNC_PORT}")
        echo "进程 ID: $NOVNC_PID"
        echo "监听端口: ${NOVNC_PORT}"
        echo "浏览器访问: http://$SERVER_IP:${NOVNC_PORT}/vnc.html"
    else
        echo -e "状态: ${RED}未运行${NC}"
    fi
    
    echo ""
    
    # Check GNOME session status
    echo -e "${BLUE}[虚拟桌面会话状态]${NC}"
    if check_virtual_desktop_running; then
        echo -e "状态: ${GREEN}运行中${NC}"
        GNOME_RUNNING=true
        
        # Try to detect which desktop environment is running
        DETECTED_ENV="未知"
        if DISPLAY="${VNC_DISPLAY}" wmctrl -l 2>/dev/null | grep -q ".*-1.*"; then
            # wmctrl shows windows, try to detect DE from window properties
            DETECTED_ENV="运行中"
        else
            DETECTED_ENV="运行中（无法确定环境）"
        fi
        
        echo "环境: $DETECTED_ENV"
        
        # Check VirtualGL
        if command -v vglrun &> /dev/null; then
            echo -e "VirtualGL: ${GREEN}已安装并启用（GPU 加速）${NC}"
        else
            echo -e "VirtualGL: ${YELLOW}未安装（软件渲染）${NC}"
        fi
    else
        echo -e "状态: ${RED}未运行${NC}"
    fi
    
    echo ""
    
    # Check ports
    echo -e "${BLUE}[端口监听状态]${NC}"
    if command -v netstat &> /dev/null; then
        netstat -tuln | grep -E ":(${VNC_PORT}|${NOVNC_PORT})" || echo "无 VNC 相关端口监听"
    elif command -v ss &> /dev/null; then
        ss -tuln | grep -E ":(${VNC_PORT}|${NOVNC_PORT})" || echo "无 VNC 相关端口监听"
    fi
    
    echo ""
    
    # Summary
    echo -e "${BLUE}[服务状态总结]${NC}"
    if [ "$XVFB_RUNNING" = true ] && [ "$X11VNC_RUNNING" = true ] && [ "$NOVNC_RUNNING" = true ] && [ "$GNOME_RUNNING" = true ]; then
        echo -e "${GREEN}● VNC 服务完全运行${NC}"
    elif [ "$XVFB_RUNNING" = true ] || [ "$X11VNC_RUNNING" = true ] || [ "$NOVNC_RUNNING" = true ] || [ "$GNOME_RUNNING" = true ]; then
        echo -e "${YELLOW}◐ VNC 服务部分运行${NC}"
        echo "   Xvfb: $([ "$XVFB_RUNNING" = true ] && echo "✓" || echo "✗")"
        echo "   x11vnc: $([ "$X11VNC_RUNNING" = true ] && echo "✓" || echo "✗")"
        echo "   noVNC: $([ "$NOVNC_RUNNING" = true ] && echo "✓" || echo "✗")"
        echo "   GNOME: $([ "$GNOME_RUNNING" = true ] && echo "✓" || echo "✗")"
    else
        echo -e "${RED}○ VNC 服务未运行${NC}"
    fi
    
    echo ""
    echo "服务器 IP: $SERVER_IP"
    echo "显示: ${VNC_DISPLAY}"
    
    if [ "$X11VNC_RUNNING" = true ] && [ "$NOVNC_RUNNING" = true ]; then
        echo ""
        echo -e "${GREEN}访问方式:${NC}"
        echo "  VNC 客户端: $SERVER_IP:${VNC_PORT}"
        echo "  浏览器: http://$SERVER_IP:${NOVNC_PORT}/vnc.html"
    fi
    
    if [ "$GNOME_RUNNING" = true ] && command -v vglrun &> /dev/null; then
        echo ""
        echo -e "${GREEN}VirtualGL GPU 加速已启用${NC}"
    fi
    echo ""
}

# Main script logic
check_vnc_installed

# Check command argument
if [ $# -eq 0 ]; then
    show_usage
fi

case "$1" in
    start)
        start_vnc
        ;;
    stop)
        stop_vnc
        ;;
    status)
        status_vnc
        ;;
    *)
        echo -e "${RED}错误: 未知命令 '$1'${NC}"
        echo ""
        show_usage
        ;;
esac

exit 0
