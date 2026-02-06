#!/bin/bash

###############################################################################
# VNC and noVNC Uninstallation Script for Ubuntu 22.04
# Supports: x86_64 and ARM architectures
# Description: Uninstalls VNC server, noVNC, and related components
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
    echo "请使用: sudo $0"
    exit 1
fi

# Get the actual user (not root)
ACTUAL_USER=${SUDO_USER:-$USER}
USER_HOME=$(eval echo ~$ACTUAL_USER)

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   VNC 和 noVNC 卸载脚本${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "用户: $ACTUAL_USER"
echo "用户主目录: $USER_HOME"
echo "脚本目录: $SCRIPT_DIR"
echo ""

# Warning prompt
echo -e "${YELLOW}警告: 此脚本将卸载以下组件:${NC}"
echo "  - Xvfb 虚拟显示服务器"
echo "  - x11vnc VNC 服务器"
echo "  - noVNC 和 websockify"
echo "  - VirtualGL (如果已安装)"
echo "  - VNC 防火墙规则"
echo ""
echo -e "${YELLOW}注意: 桌面环境 (XFCE/GNOME) 将不会被卸载${NC}"
echo -e "${YELLOW}      如需卸载桌面环境，请在卸载完成后手动操作${NC}"
echo ""

read -p "确认要继续卸载吗? [y/N]: " CONFIRM
CONFIRM=${CONFIRM:-N}

if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
    echo "取消卸载"
    exit 0
fi

echo ""
echo -e "${GREEN}开始卸载流程...${NC}"
echo ""

# Function to stop all VNC services
stop_vnc_services() {
    echo -e "${BLUE}=== 停止 VNC 相关服务 ===${NC}"
    
    # Stop processes started by vglrun (virtual desktop sessions)
    echo "停止通过 vglrun 启动的虚拟桌面会话..."
    pkill -f "vglrun.*gnome-session" 2>/dev/null
    pkill -f "vglrun.*xfce" 2>/dev/null
    pkill -f "vglrun.*startxfce4" 2>/dev/null
    
    # Stop noVNC
    echo "停止 noVNC 服务..."
    pkill -f "websockify" 2>/dev/null
    pkill -f "novnc" 2>/dev/null
    
    # Stop x11vnc
    echo "停止 x11vnc 服务..."
    pkill -f "x11vnc" 2>/dev/null
    
    # Stop Xvfb
    echo "停止 Xvfb 虚拟显示服务器..."
    pkill -f "Xvfb" 2>/dev/null
    
    # Stop TurboVNC if exists
    if command -v /opt/TurboVNC/bin/vncserver &> /dev/null; then
        echo "停止 TurboVNC 服务..."
        su - "$ACTUAL_USER" -c "/opt/TurboVNC/bin/vncserver -kill :0" 2>/dev/null
        su - "$ACTUAL_USER" -c "/opt/TurboVNC/bin/vncserver -kill :1" 2>/dev/null
    fi
    
    sleep 2
    echo -e "${GREEN}VNC 服务已停止${NC}"
    echo ""
}

# Function to clean up lock files and temporary files
cleanup_temp_files() {
    echo -e "${BLUE}=== 清理临时文件和锁文件 ===${NC}"
    
    # Remove X11 lock files
    rm -f /tmp/.X*-lock 2>/dev/null
    rm -f /tmp/.X11-unix/X* 2>/dev/null
    
    # Remove VNC log files
    rm -f /tmp/xvfb.log /tmp/x11vnc.log /tmp/novnc.log /tmp/gnome-session.log 2>/dev/null
    rm -f /var/log/novnc.log 2>/dev/null
    
    # Remove user VNC directory
    if [ -d "$USER_HOME/.vnc" ]; then
        echo "删除用户 VNC 配置目录: $USER_HOME/.vnc"
        rm -rf "$USER_HOME/.vnc"
    fi
    
    # Remove TurboVNC directory if exists
    if [ -d "$USER_HOME/.vnc" ]; then
        rm -rf "$USER_HOME/.vnc"
    fi
    
    echo -e "${GREEN}临时文件清理完成${NC}"
    echo ""
}

# Function to uninstall Xvfb and x11vnc
uninstall_xvfb_x11vnc() {
    echo -e "${BLUE}=== 卸载 Xvfb 和 x11vnc ===${NC}"
    
    if dpkg -l | grep -q "xvfb\|x11vnc"; then
        apt-get remove -y xvfb x11vnc
        apt-get autoremove -y
        echo -e "${GREEN}Xvfb 和 x11vnc 已卸载${NC}"
    else
        echo -e "${YELLOW}Xvfb 和 x11vnc 未安装，跳过${NC}"
    fi
    
    echo ""
}

# Function to remove noVNC and websockify
remove_novnc_websockify() {
    echo -e "${BLUE}=== 删除 noVNC 和 websockify ===${NC}"
    
    NOVNC_PATH="$SCRIPT_DIR/pkg/noVNC"
    
    if [ -d "$NOVNC_PATH" ]; then
        echo "删除 noVNC 目录: $NOVNC_PATH"
        rm -rf "$NOVNC_PATH"
        echo -e "${GREEN}noVNC 和 websockify 已删除${NC}"
    else
        echo -e "${YELLOW}noVNC 目录不存在，跳过${NC}"
    fi
    
    # Also check for system-wide noVNC installation
    if [ -d "/usr/share/novnc" ]; then
        echo "检测到系统级 noVNC 安装"
        read -p "是否删除系统级 noVNC? [y/N]: " REMOVE_SYSTEM_NOVNC
        REMOVE_SYSTEM_NOVNC=${REMOVE_SYSTEM_NOVNC:-N}
        
        if [[ "$REMOVE_SYSTEM_NOVNC" =~ ^[Yy]$ ]]; then
            apt-get remove -y novnc 2>/dev/null || rm -rf /usr/share/novnc
            echo -e "${GREEN}系统级 noVNC 已删除${NC}"
        fi
    fi
    
    echo ""
}

# Function to uninstall VirtualGL
uninstall_virtualgl() {
    echo -e "${BLUE}=== 卸载 VirtualGL ===${NC}"
    
    if command -v vglrun &> /dev/null; then
        echo "检测到 VirtualGL，正在卸载..."
        
        # Try to remove using dpkg
        dpkg -r virtualgl 2>/dev/null || apt-get remove -y virtualgl 2>/dev/null
        
        # Remove VirtualGL files manually if still exists
        if command -v vglrun &> /dev/null; then
            rm -rf /opt/VirtualGL 2>/dev/null
            rm -f /usr/bin/vglrun /usr/bin/vglconnect /usr/bin/vglclient 2>/dev/null
        fi
        
        echo -e "${GREEN}VirtualGL 已卸载${NC}"
    else
        echo -e "${YELLOW}VirtualGL 未安装，跳过${NC}"
    fi
    
    echo ""
}

# Function to remove firewall rules
remove_firewall_rules() {
    echo -e "${BLUE}=== 删除防火墙规则 ===${NC}"
    
    if command -v ufw &> /dev/null; then
        UFW_STATUS=$(ufw status 2>/dev/null | grep -i "Status:" | awk '{print $2}')
        
        if [ "$UFW_STATUS" = "active" ]; then
            echo "删除 VNC 相关防火墙规则..."
            
            # Remove VNC ports
            ufw delete allow 5900/tcp 2>/dev/null
            ufw delete allow 5901/tcp 2>/dev/null
            
            # Remove noVNC ports
            ufw delete allow 6080/tcp 2>/dev/null
            ufw delete allow 8080/tcp 2>/dev/null
            
            echo -e "${GREEN}防火墙规则已删除${NC}"
        else
            echo -e "${YELLOW}防火墙未启动，无需删除规则${NC}"
        fi
    else
        echo -e "${YELLOW}未检测到 ufw 防火墙，跳过${NC}"
    fi
    
    echo ""
}

# Function to remove systemd services if exists
remove_systemd_services() {
    echo -e "${BLUE}=== 删除 systemd 服务 (如果存在) ===${NC}"
    
    # Check for VNC related systemd services
    if systemctl list-unit-files | grep -q "turbovnc\|novnc\|vncserver"; then
        echo "检测到 VNC 相关 systemd 服务，正在删除..."
        
        systemctl stop turbovncserver@*.service 2>/dev/null
        systemctl disable turbovncserver@*.service 2>/dev/null
        
        systemctl stop novnc@*.service 2>/dev/null
        systemctl disable novnc@*.service 2>/dev/null
        
        # Remove service files
        rm -f /etc/systemd/system/turbovncserver@.service 2>/dev/null
        rm -f /etc/systemd/system/novnc@.service 2>/dev/null
        
        systemctl daemon-reload
        
        echo -e "${GREEN}systemd 服务已删除${NC}"
    else
        echo -e "${YELLOW}未检测到 VNC 相关 systemd 服务，跳过${NC}"
    fi
    
    echo ""
}

# Function to show desktop environment info
show_desktop_info() {
    echo ""
    echo -e "${BLUE}=== 桌面环境状态 ===${NC}"
    
    if dpkg -l | grep -q "^ii.*xfce4[[:space:]]"; then
        echo -e "${GREEN}XFCE 桌面环境: 已保留${NC}"
        echo "如需卸载，请运行: sudo apt-get remove --purge xfce4 xfce4-goodies"
    fi
    
    if dpkg -l | grep -q "^ii.*gnome-shell[[:space:]]"; then
        echo -e "${GREEN}GNOME 桌面环境: 已保留${NC}"
        echo "如需卸载，请运行: sudo apt-get remove --purge ubuntu-desktop gnome-shell gnome-session"
    fi
    
    if dpkg -l | grep -q "^ii.*plasma-desktop[[:space:]]"; then
        echo -e "${GREEN}KDE Plasma 桌面环境: 已保留${NC}"
        echo "如需卸载，请运行: sudo apt-get remove --purge kde-plasma-desktop"
    fi
    
    echo ""
}

###############################################################################
# 主卸载流程
###############################################################################

# Stop all VNC services
stop_vnc_services

# Clean up temporary files
cleanup_temp_files

# Uninstall components
uninstall_xvfb_x11vnc
remove_novnc_websockify
uninstall_virtualgl

# Remove firewall rules
remove_firewall_rules

# Remove systemd services
remove_systemd_services

# Clean up remaining packages
echo -e "${BLUE}=== 清理残留软件包 ===${NC}"
apt-get autoremove -y
apt-get autoclean
echo -e "${GREEN}软件包清理完成${NC}"
echo ""

# Show desktop environment info
show_desktop_info

# Summary
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   VNC 卸载完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${GREEN}已卸载的组件:${NC}"
echo "  ✓ Xvfb 虚拟显示服务器"
echo "  ✓ x11vnc VNC 服务器"
echo "  ✓ noVNC 和 websockify"
echo "  ✓ VirtualGL (如果已安装)"
echo "  ✓ VNC 防火墙规则"
echo "  ✓ 临时文件和锁文件"
echo ""
echo -e "${BLUE}保留的组件:${NC}"
echo "  • 桌面环境 (XFCE/GNOME/KDE)"
echo ""
echo -e "${YELLOW}提示:${NC}"
echo "  - 桌面环境已保留，如需卸载请参考上面的命令"
echo "  - 如果遇到问题，可能需要重启系统"
echo ""

exit 0
