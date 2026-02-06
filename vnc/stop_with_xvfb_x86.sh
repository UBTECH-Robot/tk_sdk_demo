#!/bin/bash

# 停止 start_with_xvfb.sh 启动的VNC服务
# 此脚本确保彻底清理虚拟显示 :99 上的所有进程，包括 gnome-session

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

# 停止 gnome-session (虚拟显示:99)
# 关键是检查进程的环境变量中的 DISPLAY=:99
echo "停止 GNOME 组件 (:99)..."
GNOME_KILLED=0

# 只通过环境变量精确检查虚拟显示 :99 上的进程
for component in gnome-session gnome-shell mutter metacity openbox; do
    for pid in $(pgrep -f "$component" 2>/dev/null); do
        if [ -r "/proc/$pid/environ" ]; then
            # 严格检查：环境变量用\0分隔，需要转换为\n后grep
            # 只有 DISPLAY 环境变量值恰好为 ":99" 的进程才清理
            if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -xF "DISPLAY=:99" >/dev/null 2>&1; then
                # 杀死该进程及其所有子进程
                pkill -9 -P "$pid" 2>/dev/null || true
                kill -9 "$pid" 2>/dev/null || true
                echo "✓ $component (PID: $pid) 已停止"
                GNOME_KILLED=$((GNOME_KILLED + 1))
            fi
        fi
    done
done

if [ $GNOME_KILLED -eq 0 ]; then
    echo "✗ GNOME 组件 (:99) 未运行"
else
    KILLED=$((KILLED + GNOME_KILLED))
    echo "✓ 已停止 $GNOME_KILLED 个 GNOME 相关进程"
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

# 停止虚拟X服务器 (Xvfb :99)
echo "停止虚拟X服务器 (Xvfb :99)..."
XVFB_PID=$(pgrep -f "Xvfb.*:99" | head -1)
if [ -n "$XVFB_PID" ]; then
    # 先杀死 Xvfb 的所有子进程
    pkill -9 -P "$XVFB_PID" 2>/dev/null || true
    # 再杀死 Xvfb 本身
    kill -9 "$XVFB_PID" 2>/dev/null || true
    echo "✓ Xvfb 已停止"
    KILLED=$((KILLED + 1))
else
    echo "✗ Xvfb 未运行"
fi

# 等待进程完全退出
sleep 2

# 最终清理：再次尝试杀死任何残留的虚拟显示 :99 进程（仅通过精确环境变量检查）
echo ""
echo "进行最终清理..."
FINAL_KILLED=0

# 最后一次扫描：确保没有遗留的虚拟显示进程
for pid in $(ps aux | awk '{print $2}' | sort -u); do
    if [ -r "/proc/$pid/environ" ]; then
        if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -xF "DISPLAY=:99" >/dev/null 2>&1; then
            # 这个进程的 DISPLAY 是 :99，需要杀死
            if ! echo "$pid" | grep -qE "Xvfb|x11vnc|novnc" 2>/dev/null; then
                # 不是已经处理过的关键进程
                pkill -9 -P "$pid" 2>/dev/null || true
                kill -9 "$pid" 2>/dev/null || true
                FINAL_KILLED=$((FINAL_KILLED + 1))
            fi
        fi
    fi
done 2>/dev/null || true

if [ $FINAL_KILLED -gt 0 ]; then
    echo "✓ 清理了 $FINAL_KILLED 个残留进程"
    sleep 1
fi

# 检查并清理占用的端口
echo ""
echo "检查端口释放状态..."
if ss -tulpn 2>/dev/null | grep -E ":5900|:8080" >/dev/null 2>&1; then
    echo "⚠ 某些进程仍然占用端口，进行强制清理..."
    for port in 5900 8080; do
        if ss -tulpn 2>/dev/null | grep ":$port" >/dev/null 2>&1; then
            pids=$(ss -tulpn 2>/dev/null | grep ":$port" | awk -F'pid=' '{print $NF}' | awk '{print $1}' | sort -u)
            if [ -n "$pids" ]; then
                echo "杀死占用端口 $port 的进程..."
                echo "$pids" | xargs -r kill -9 2>/dev/null || true
            fi
        fi
    done
    sleep 1
fi

# 验证端口已释放
if ! ss -tulpn 2>/dev/null | grep -E ":5900|:8080"; then
    echo "✓ 端口已释放"
else
    echo "⚠ 端口仍在使用（可能需要重新启动系统或手动检查）"
fi

# 清理临时文件
echo ""
echo "清理临时文件..."
rm -f /tmp/xvfb.Xauth 2>/dev/null && echo "✓ 临时Xauth已清理" || true

# 最终验证：只检查关键进程
echo ""
echo "最终验证..."

# 检查虚拟X服务器和VNC服务
XVFB_REMAINING=$(pgrep -f "Xvfb.*:99" 2>/dev/null | wc -l)
X11VNC_REMAINING=$(pgrep -f "x11vnc.*:99" 2>/dev/null | wc -l)
NOVNC_REMAINING=$(pgrep -f "novnc_proxy" 2>/dev/null | wc -l)

TOTAL_REMAINING=$((XVFB_REMAINING + X11VNC_REMAINING + NOVNC_REMAINING))

if [ "$TOTAL_REMAINING" -eq 0 ]; then
    echo "=========================================="
    echo "✓ VNC 虚拟桌面服务已完全停止"
    echo "=========================================="
else
    echo "⚠ 仍有 $TOTAL_REMAINING 个虚拟显示相关进程"
    echo "=========================================="
fi
echo ""
