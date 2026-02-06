#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 安装依赖
sudo apt install xvfb x11vnc -y
cd "$SCRIPT_DIR/pkg" && sudo dpkg -i virtualgl_3.1_amd64.deb 
cd "$SCRIPT_DIR/pkg" && tar -zxf noVNC-1.6.0.tar.gz && mv noVNC-1.6.0 noVNC
cd "$SCRIPT_DIR/pkg" && tar -zxf websockify-0.13.0.tar.gz && mv websockify-0.13.0 noVNC/pkg/websockify

# 启动虚拟桌面环境
export DISPLAY=":0"
export XDG_SESSION_TYPE=x11
export GDK_BACKEND=x11

Xvfb "${DISPLAY}" -ac -screen "0" "1920x1080x24" -dpi "96" +extension "RANDR" +extension "GLX" +iglx +extension "MIT-SHM" +render -nolisten "tcp" -noreset -shmem &

echo "Waiting for X socket"
until [ -S "/tmp/.X11-unix/X${DISPLAY/:/}" ]; do sleep 1; done
echo "X socket is ready"

x11vnc -display "${DISPLAY}" -shared -nomodtweak -forever -capslock -repeat -xkb -xrandr "resize" -rfbport 5900 -noshm &

./pkg/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 8080 --heartbeat 10 &

export VGL_DISPLAY="egl"
export VGL_REFRESHRATE="60"
vglrun +wm gnome-session &
# gnome-session &