#!/bin/bash

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

# vglrun +wm gnome-session &
gnome-session &