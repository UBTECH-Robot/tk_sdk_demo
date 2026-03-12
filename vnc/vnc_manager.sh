#!/bin/bash

# VNC 服务管理脚本
# 用法: ./vnc_manager.sh {start|stop|restart|status}
# 只支持 xvfb 虚拟桌面模式（最稳定有效）

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMAND="${1:-status}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

if [ -n "${SUDO_USER:-}" ]; then
    RUNNER=(sudo bash)
    RUN_MODE="sudo"
else
    RUNNER=(bash)
    RUN_MODE="普通"
fi

show_usage() {
    echo "用法: $0 {start|stop|restart|status}"
    echo ""
    echo "命令:"
    echo "  start   - 启动VNC虚拟桌面服务"
    echo "  stop    - 停止VNC虚拟桌面服务"
    echo "  restart - 重启VNC虚拟桌面服务"
    echo "  status  - 查看VNC服务状态"
    echo ""
    echo "示例:"
    echo "  $0 start   # 启动VNC"
    echo "  $0 stop    # 停止VNC"
    echo "  $0 status  # 查看状态"
}

show_status() {
    echo "========================================"
    echo "VNC 服务状态"
    echo "========================================"
    echo ""
    echo "当前运行模式: ${RUN_MODE}模式"
    echo ""
    
    # 检查x11vnc
    if pgrep -f "x11vnc" > /dev/null; then
        echo -e "${GREEN}✓${NC} x11vnc 运行中"
        pgrep -f "x11vnc" | while read pid; do
            ps -p $pid -o cmd= | sed 's/^/  /'
        done
    else
        echo -e "${RED}✗${NC} x11vnc 未运行"
    fi
    
    # 检查noVNC
    if pgrep -f "novnc_proxy" > /dev/null; then
        echo -e "${GREEN}✓${NC} noVNC 运行中"
        pgrep -f "novnc_proxy" | while read pid; do
            ps -p $pid -o cmd= | sed 's/^/  /'
        done
    else
        echo -e "${RED}✗${NC} noVNC 未运行"
    fi
    
    # 检查Xvfb
    if pgrep -f "Xvfb" > /dev/null; then
        echo -e "${GREEN}✓${NC} Xvfb 运行中"
        pgrep -f "Xvfb" | while read pid; do
            ps -p $pid -o cmd= | sed 's/^/  /'
        done
    else
        echo -e "${RED}✗${NC} Xvfb 未运行"
    fi
    
    # 检查端口
    echo ""
    echo "端口状态:"
    if ss -tulpn 2>/dev/null | grep -q ":5900"; then
        echo -e "${GREEN}✓${NC} 5900 (VNC) 开放"
    else
        echo -e "${RED}✗${NC} 5900 (VNC) 未开放"
    fi
    
    if ss -tulpn 2>/dev/null | grep -q ":8080"; then
        echo -e "${GREEN}✓${NC} 8080 (noVNC Web) 开放"
    else
        echo -e "${RED}✗${NC} 8080 (noVNC Web) 未开放"
    fi
    
    # 显示访问地址
    IP_ADDR=$(hostname -I | awk '{print $1}')
    if [ -n "$IP_ADDR" ]; then
        echo ""
        echo "访问地址:"
        echo -e "  Web: ${BLUE}http://${IP_ADDR}:8080${NC}"
        echo -e "  VNC: ${BLUE}${IP_ADDR}:5900${NC}"
    fi
    
    echo "========================================"
}

start_service() {
    echo -e "${BLUE}启动 VNC 虚拟桌面服务...${NC}"
    echo "当前运行模式: ${RUN_MODE}模式"
    "${RUNNER[@]}" "$SCRIPT_DIR/start_with_xvfb.sh"
}

stop_service() {
    echo -e "${BLUE}停止 VNC 虚拟桌面服务...${NC}"
    echo "当前运行模式: ${RUN_MODE}模式"
    "${RUNNER[@]}" "$SCRIPT_DIR/stop_with_xvfb.sh"
}

restart_service() {
    stop_service
    sleep 2
    start_service
}

# 执行命令
case "$COMMAND" in
    start)
        start_service
        ;;
    stop)
        stop_service
        ;;
    restart)
        restart_service
        ;;
    status)
        show_status
        ;;
    *)
        show_usage
        exit 1
        ;;
esac
