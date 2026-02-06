#!/bin/bash

# 虚拟X服务器 + 完整桌面环境方案
# 此方案创建一个虚拟显示器并在其上运行完整的桌面环境

DISPLAY=":99"
SCREEN_WIDTH="1920"
SCREEN_HEIGHT="1080"
VNC_PORT="5900"
NOVNC_PORT="8080"

echo "========================================"
echo "VNC服务启动 (虚拟桌面模式)"
echo "========================================"
echo "虚拟显示器: ${DISPLAY}"
echo "分辨率: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
echo ""

# 清理旧的显示
pkill -f "Xvfb.*${DISPLAY}" 2>/dev/null
pkill -f "x11vnc.*${DISPLAY}" 2>/dev/null
sleep 1

# 启动虚拟X服务器
echo "启动虚拟X服务器 (Xvfb)..."
Xvfb "${DISPLAY}" \
    -screen 0 "${SCREEN_WIDTH}x${SCREEN_HEIGHT}x24" \
    -dpi 96 \
    +extension "RANDR" \
    +extension "GLX" \
    +iglx \
    +extension "MIT-SHM" \
    -nolisten "tcp" \
    -noreset \
    -shmem &

XVFB_PID=$!
echo "Xvfb PID: ${XVFB_PID}"

# 等待Xvfb启动
sleep 2

# 检查Xvfb是否成功
if ! kill -0 ${XVFB_PID} 2>/dev/null; then
    echo "错误: Xvfb启动失败"
    exit 1
fi

echo "✓ Xvfb 启动成功"

# 设置环境变量
export DISPLAY="${DISPLAY}"
export XAUTHORITY="/tmp/xvfb.Xauth"

# 为虚拟显示器创建Xauthority
xauth add "${DISPLAY}" . "$(xxd -l 16 -p /dev/urandom)" 2>/dev/null || true

# 启动窗口管理器 (使用 openbox 如果可用，否则使用 fluxbox)
echo ""
echo "启动窗口管理器..."

if command -v openbox &> /dev/null; then
    openbox --display "${DISPLAY}" &
    WM_PID=$!
    echo "启动 openbox (PID: ${WM_PID})"
elif command -v fluxbox &> /dev/null; then
    fluxbox -display "${DISPLAY}" &
    WM_PID=$!
    echo "启动 fluxbox (PID: ${WM_PID})"
elif command -v mwm &> /dev/null; then
    mwm -display "${DISPLAY}" &
    WM_PID=$!
    echo "启动 mwm (PID: ${WM_PID})"
else
    echo "警告: 未找到窗口管理器，使用 x11vnc 的内置支持"
    WM_PID=""
fi

sleep 1

# 启动壁纸和其他X应用 (可选)
DISPLAY="${DISPLAY}" xsetroot -solid "#1e1e2e" 2>/dev/null || true

# 启动x11vnc
echo ""
echo "启动 x11vnc..."
x11vnc -display "${DISPLAY}" \
    -auth "${XAUTHORITY}" \
    -shared \
    -nomodtweak \
    -forever \
    -capslock \
    -repeat \
    -xkb \
    -xrandr "resize" \
    -rfbport ${VNC_PORT} \
    -noshm \
    -sb 10000 \
    -logfile /tmp/x11vnc.log &

X11VNC_PID=$!
echo "x11vnc PID: ${X11VNC_PID}"

sleep 2

if ! kill -0 ${X11VNC_PID} 2>/dev/null; then
    echo "错误: x11vnc启动失败"
    cat /tmp/x11vnc.log
    exit 1
fi

echo "✓ x11vnc 启动成功"

# 启动noVNC Web代理
echo ""
echo "启动 noVNC..."
cd "$(dirname "$0")"

./pkg/noVNC/utils/novnc_proxy \
    --vnc localhost:${VNC_PORT} \
    --listen 0.0.0.0:${NOVNC_PORT} \
    --file-only \
    --heartbeat 10 2>&1 &

NOVNC_PID=$!
echo "noVNC PID: ${NOVNC_PID}"

sleep 2

if ! kill -0 ${NOVNC_PID} 2>/dev/null; then
    echo "错误: noVNC启动失败"
    exit 1
fi

echo "✓ noVNC 启动成功"

echo ""
echo "启动GNOME桌面..."
export VGL_DISPLAY="egl"
export VGL_REFRESHRATE="60"

# 检查是否有vglrun命令
if command -v vglrun &> /dev/null; then
    echo "使用 vglrun 启动 gnome-session..."
    vglrun +wm gnome-session &
    GNOME_PID=$!
else
    echo "vglrun 不可用，直接启动 gnome-session..."
    gnome-session &
    GNOME_PID=$!
fi

sleep 1

# 获取IP地址
IP_ADDR=$(hostname -I | awk '{print $1}')

echo ""
echo "=========================================="
echo "✓ VNC虚拟桌面已启动!"
echo "=========================================="
echo "Web访问地址: http://localhost:${NOVNC_PORT}"
echo "            http://${IP_ADDR}:${NOVNC_PORT}"
echo ""
echo "VNC直连地址: localhost:${VNC_PORT}"
echo "            ${IP_ADDR}:${VNC_PORT}"
echo ""
echo "分辨率: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
echo "x11vnc 日志: /tmp/x11vnc.log"
echo "=========================================="
echo ""

# 清理函数
cleanup() {
    echo ""
    echo "停止VNC服务..."
    kill ${NOVNC_PID} 2>/dev/null
    kill ${X11VNC_PID} 2>/dev/null
    [ -n "${GNOME_PID}" ] && kill ${GNOME_PID} 2>/dev/null
    [ -n "${WM_PID}" ] && kill ${WM_PID} 2>/dev/null
    [ -n "${TERM_PID}" ] && kill ${TERM_PID} 2>/dev/null
    kill ${XVFB_PID} 2>/dev/null
    wait
}

trap cleanup TERM INT EXIT

# 保留进程运行
wait
