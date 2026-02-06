#!/bin/bash

# 停止 start_with_xvfb.sh 启动的VNC服务

echo "========================================"
echo "停止 VNC 虚拟桌面服务"
echo "========================================"
echo ""

# 计数器
KILLED=0

# 停止 noVNC
echo "停止 noVNC..."
if pkill -f "novnc_proxy" 2>/dev/null; then
    echo "✓ noVNC 已停止"
    KILLED=$((KILLED + 1))
else
    echo "✗ noVNC 未运行"
fi

# 停止 x11vnc (虚拟显示:99)
echo "停止 x11vnc (:99)..."
if pkill -f "x11vnc.*:99" 2>/dev/null; then
    echo "✓ x11vnc 已停止"
    KILLED=$((KILLED + 1))
else
    echo "✗ x11vnc 未运行"
fi

# 停止窗口管理器
echo "停止窗口管理器..."
if pkill -f "openbox.*:99" 2>/dev/null; then
    echo "✓ openbox 已停止"
    KILLED=$((KILLED + 1))
elif pkill -f "fluxbox.*:99" 2>/dev/null; then
    echo "✓ fluxbox 已停止"
    KILLED=$((KILLED + 1))
elif pkill -f "mwm.*:99" 2>/dev/null; then
    echo "✓ mwm 已停止"
    KILLED=$((KILLED + 1))
else
    echo "✗ 窗口管理器未运行"
fi

# 停止 Xvfb
echo "停止虚拟X服务器 (Xvfb :99)..."
if pkill -f "Xvfb.*:99" 2>/dev/null; then
    echo "✓ Xvfb 已停止"
    KILLED=$((KILLED + 1))
else
    echo "✗ Xvfb 未运行"
fi

# 等待进程完全退出
sleep 2

# 检查端口是否释放
echo ""
echo "检查端口释放状态..."
if ss -tulpn 2>/dev/null | grep -q ":5900" || ss -tulpn 2>/dev/null | grep -q ":8080"; then
    echo "⚠ 某些进程仍然占用端口，进行强制杀死..."
    sudo lsof -i :5900 -i :8080 2>/dev/null | grep -v COMMAND | awk '{print $2}' | sort -u | xargs -r sudo kill -9 2>/dev/null
    sleep 1
fi

# 验证端口已释放
if ! ss -tulpn 2>/dev/null | grep -E ":5900|:8080"; then
    echo "✓ 端口已释放"
else
    echo "⚠ 端口仍在使用"
fi

# 清理临时文件 (可选)
echo ""
echo "清理临时文件..."
rm -f /tmp/xvfb.Xauth 2>/dev/null && echo "✓ 临时Xauth已清理" || true

echo ""
echo "========================================"
echo "✓ VNC 虚拟桌面服务已停止"
echo "========================================"
echo ""
