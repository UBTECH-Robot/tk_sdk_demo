#!/bin/bash

# 虚拟X服务器 + 完整桌面环境方案
# 此方案创建一个虚拟显示器并在其上运行完整的桌面环境
# 特别注意：禁用登录界面，直接启动桌面

DISPLAY=":99"
SCREEN_WIDTH="1920"
SCREEN_HEIGHT="1080"
VNC_PORT="5900"
NOVNC_PORT="8888"

echo "========================================"
echo "VNC服务启动 (虚拟桌面模式)"
echo "========================================"
echo "虚拟显示器: ${DISPLAY}"
echo "分辨率: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
echo ""

# 清理旧的显示
echo "清理旧的显示..."
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
    -logfile ./x11vnc.log &

X11VNC_PID=$!
echo "x11vnc PID: ${X11VNC_PID}"

sleep 2

if ! kill -0 ${X11VNC_PID} 2>/dev/null; then
    echo "错误: x11vnc启动失败"
    cat ./x11vnc.log
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
export DISPLAY="${DISPLAY}"
export XAUTHORITY="/tmp/xvfb.Xauth"

# 设置 X11 会话（避免 Wayland 导致的问题）
export XDG_SESSION_TYPE=x11

# 初始化 D-Bus 会话
eval $(dbus-launch --sh-syntax 2>/dev/null) || true
export DBUS_SYSTEM_BUS_ADDRESS=unix:path=/var/run/dbus/system_bus_socket 2>/dev/null || true

# 禁用 GNOME 屏保和自动锁屏，避免登录界面
dconf write /org/gnome/desktop/screensaver/lock-enabled false 2>/dev/null || true
dconf write /org/gnome/desktop/session/idle-delay 0 2>/dev/null || true

# 禁用 GNOME Initial Setup（初始设置向导）
# 在 ARM 上需要明确禁用，在 x86 上也要确保不出现
echo "禁用 GNOME Initial Setup..."

# 方法1：创建标记文件（系统级）
sudo touch /var/lib/gnome-initial-setup-done 2>/dev/null || true
sudo touch /etc/gnome-initial-setup-done 2>/dev/null || true

# 方法2：创建用户级标记文件
mkdir -p ~/.config/gnome-shell 2>/dev/null || true
touch ~/.config/gnome-initial-setup-done 2>/dev/null || true
touch ~/.config/gnome-shell/disabled-extensions 2>/dev/null || true

# 方法3：使用 dconf 禁用
DISPLAY="${DISPLAY}" dconf write /org/gnome/initial-setup/run false 2>/dev/null || true

# 方法4：禁用 welcome 应用
DISPLAY="${DISPLAY}" gsettings set org.gnome.welcome show-on-login false 2>/dev/null || true

# 方法5：设置环境变量
export GNOME_INITIAL_SETUP=0

echo "启动 GNOME 会话..."

# 仅在 VNC 会话期间压制 apport，避免弹出 "System program problem detected" 对话框
# 只 stop 不修改配置，VNC 停止后由 stop_with_xvfb.sh 恢复
sudo systemctl stop apport 2>/dev/null || true
# 清除已有的崩溃报告，避免 GNOME 启动时重新弹出
sudo rm -f /var/crash/*.crash 2>/dev/null || true

# 若 vglrun 可用，将 vgl_wrappers 目录注入 PATH 头部，劫持 ros2 / rviz2 → vglrun
# GNOME 会话所有子进程（终端、桌面启动器、脚本）均继承此 PATH
if command -v vglrun >/dev/null 2>&1; then
    _VGL_WRAP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/vgl_wrappers"
    chmod +x "$_VGL_WRAP_DIR"/ros2 "$_VGL_WRAP_DIR"/rviz2 2>/dev/null || true
    export PATH="${_VGL_WRAP_DIR}:${PATH}"
    echo "✓ vglrun 可用，已启用 ros2/rviz2 → vglrun 劫持"
else
    echo "✗ vglrun 未安装，跳过 ros2/rviz2 劫持（OpenGL 渲染走软件模式）"
fi

# 直接使用 gnome-session-binary 启动，跳过登录管理器
if [ -f /usr/libexec/gnome-session-binary ]; then
    DISPLAY="${DISPLAY}" XAUTHORITY="/tmp/xvfb.Xauth" \
        /usr/libexec/gnome-session-binary --session=ubuntu 2>/dev/null &
    GNOME_PID=$!
    echo "GNOME Session PID: ${GNOME_PID}"
    sleep 3
else
    DISPLAY="${DISPLAY}" gnome-session 2>/dev/null &
    GNOME_PID=$!
    echo "GNOME Session PID: ${GNOME_PID}"
    sleep 2
fi

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
echo "x11vnc 日志: ./x11vnc.log"
echo "若无权限，请停止服务后执行 sudo usermod -aG vglusers \$USER 添加当前用户到 vglusers 组，然后重新启动服务。"
echo "=========================================="
echo ""

# 清理函数
cleanup() {
    echo ""
    echo "停止VNC服务..."
    kill ${NOVNC_PID} 2>/dev/null
    kill ${X11VNC_PID} 2>/dev/null
    [ -n "${GNOME_PID}" ] && kill ${GNOME_PID} 2>/dev/null
    kill ${XVFB_PID} 2>/dev/null
    wait
}

trap cleanup TERM INT EXIT

# 保留进程运行
wait
