#!/bin/bash

###############################################################################
# VNC and noVNC Installation Script for Ubuntu 22.04
# Supports: x86_64 and ARM architectures
# Description: Installs desktop environment, VNC server, and noVNC
###############################################################################

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=../lib/apt_mirror_utils.sh
source "$SCRIPT_DIR/../lib/apt_mirror_utils.sh"

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
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

echo -e "${GREEN}=== VNC 和 noVNC 安装脚本 ===${NC}"
echo "用户: $ACTUAL_USER"
echo "用户主目录: $USER_HOME"

# Detect architecture
ARCH=$(uname -m)
echo -e "${GREEN}检测到系统架构: $ARCH${NC}"

if [[ "$ARCH" != "x86_64" && "$ARCH" != "aarch64" && "$ARCH" != "armv7l" ]]; then
    echo -e "${YELLOW}警告: 未识别的架构 $ARCH，脚本可能无法正常工作${NC}"
fi

# Function to detect GPU
detect_gpu() {
    echo -e "${GREEN}检测 GPU 硬件...${NC}"
    
    HAS_GPU=false
    GPU_TYPE=""
    IS_JETSON=false
    
    # Check for NVIDIA GPU
    if lspci 2>/dev/null | grep -i "vga\|3d\|display" | grep -iq "nvidia"; then
        HAS_GPU=true
        GPU_TYPE="NVIDIA"
        echo -e "${GREEN}检测到 NVIDIA GPU${NC}"
    fi
    
    # Check for AMD GPU
    if lspci 2>/dev/null | grep -i "vga\|3d\|display" | grep -iq "amd\|ati\|radeon"; then
        HAS_GPU=true
        GPU_TYPE="AMD"
        echo -e "${GREEN}检测到 AMD GPU${NC}"
    fi
    
    # Check for Intel GPU
    if lspci 2>/dev/null | grep -i "vga\|3d\|display" | grep -iq "intel"; then
        HAS_GPU=true
        if [ "$GPU_TYPE" = "" ]; then
            GPU_TYPE="Intel"
        else
            GPU_TYPE="${GPU_TYPE}/Intel"
        fi
        echo -e "${GREEN}检测到 Intel 集成显卡${NC}"
    fi
    
    # Special check for NVIDIA Jetson (ARM)
    if [[ "$ARCH" == "aarch64" ]]; then
        if [ -f /etc/nv_tegra_release ]; then
            IS_JETSON=true
            HAS_GPU=true
            GPU_TYPE="NVIDIA Tegra (Jetson)"
            
            # Try to get Jetson model
            if [ -f /proc/device-tree/model ]; then
                JETSON_MODEL=$(cat /proc/device-tree/model 2>/dev/null | tr -d '\0')
                echo -e "${GREEN}检测到 Jetson 设备: $JETSON_MODEL${NC}"
            else
                echo -e "${GREEN}检测到 NVIDIA Jetson 设备${NC}"
            fi
            
            # Check if it's Jetson Orin
            if grep -qi "orin" /proc/device-tree/model 2>/dev/null || grep -qi "orin" /etc/nv_tegra_release 2>/dev/null; then
                echo -e "${YELLOW}检测到 Jetson Orin 系列${NC}"
            fi
        fi
    fi
    
    if [ "$HAS_GPU" = false ]; then
        echo -e "${YELLOW}未检测到独立 GPU，将使用 CPU 软件渲染${NC}"
        echo "运行 'lspci | grep -i vga' 查看显卡信息"
    fi
}

# Function to install VirtualGL
install_virtualgl() {
    echo ""
    echo -e "${GREEN}=== 安装 VirtualGL ===${NC}"
    
    # Check if VirtualGL is already installed
    if command -v vglrun &> /dev/null; then
        echo -e "${GREEN}VirtualGL 已安装，跳过安装${NC}"
        vglrun --version 2>&1 | head -n 1 || true
        return 0
    fi
    
    if [ "$HAS_GPU" = false ]; then
        echo -e "${YELLOW}未检测到 GPU，跳过 VirtualGL 安装${NC}"
        echo "VirtualGL 需要硬件 GPU 支持才能工作"
        return 0
    fi
    
    # Determine VirtualGL package based on architecture
    if [[ "$ARCH" == "x86_64" ]]; then
        VGL_PACKAGE="virtualgl_3.1.4_amd64.deb"
    elif [[ "$ARCH" == "aarch64" ]]; then
        VGL_PACKAGE="virtualgl_3.1.4_arm64.deb"
    else
        echo -e "${RED}错误: 不支持的架构 $ARCH，无法安装 VirtualGL${NC}"
        return 1
    fi
    
    VGL_PATH="$SCRIPT_DIR/pkg/$VGL_PACKAGE"
    
    if [ -f "$VGL_PATH" ]; then
        echo "使用 VirtualGL 安装包: $VGL_PACKAGE"
        dpkg -i "$VGL_PATH" || apt-get install -f -y
        VIRTUALGL_INSTALLED=$?
        if [ $VIRTUALGL_INSTALLED -eq 0 ]; then
            echo -e "${GREEN}VirtualGL 安装成功${NC}"
        else
            echo -e "${RED}VirtualGL 安装失败${NC}"
            return 1
        fi
    else
        echo -e "${RED}警告: 找不到 $VGL_PACKAGE (路径: $VGL_PATH)${NC}"
        echo -e "${YELLOW}提示: 确保 pkg 目录中存在该文件${NC}"
        VIRTUALGL_INSTALLED=1
        return 1
    fi
    
    # Test VirtualGL installation
    if command -v vglrun &> /dev/null; then
        # Show VirtualGL info
        echo ""
        echo "VirtualGL 版本:"
        vglrun --version 2>&1 | head -n 1 || true
        
        echo ""
        echo -e "${YELLOW}提示: OpenGL 信息将在 VNC 启动后可用${NC}"
        echo "启动 VNC 后可运行: glxinfo | grep 'OpenGL renderer'"
        
        return 0
    else
        echo -e "${RED}VirtualGL 安装失败${NC}"
        return 1
    fi
}

# Function to check and install desktop environment
check_and_install_desktop() {
    echo -e "${GREEN}检查桌面环境...${NC}"
    
    DESKTOP_INSTALLED=false
    DESKTOP_TYPE=""
    
    # Check for various desktop environments
    if dpkg -l | grep -q "^ii.*xfce4[[:space:]]"; then
        echo -e "${GREEN}检测到已安装 XFCE 桌面环境${NC}"
        DESKTOP_INSTALLED=true
        DESKTOP_TYPE="xfce"
    elif dpkg -l | grep -q "^ii.*gnome-shell[[:space:]]"; then
        echo -e "${GREEN}检测到已安装 GNOME 桌面环境${NC}"
        DESKTOP_INSTALLED=true
        DESKTOP_TYPE="gnome"
    elif dpkg -l | grep -q "^ii.*plasma-desktop[[:space:]]"; then
        echo -e "${GREEN}检测到已安装 KDE Plasma 桌面环境${NC}"
        DESKTOP_INSTALLED=true
        DESKTOP_TYPE="kde"
    elif dpkg -l | grep -q "lxde\|mate-desktop"; then
        echo -e "${GREEN}检测到已安装桌面环境${NC}"
        DESKTOP_INSTALLED=true
        DESKTOP_TYPE="other"
    fi
    
    # Install desktop if no desktop is found
    if [ "$DESKTOP_INSTALLED" = false ]; then
        echo ""
        echo -e "${YELLOW}未检测到桌面环境，需要安装桌面系统${NC}"
        echo ""
        echo "请选择要安装的桌面环境:"
        echo "1) XFCE    - 轻量级，VNC性能最佳 [推荐用于VNC远程访问]"
        echo "2) GNOME   - Ubuntu 22.04默认桌面，功能丰富但较重"
        echo ""
        echo -e "${GREEN}推荐选择 1 (XFCE)${NC} - 更适合VNC远程访问，流畅稳定"
        echo -e "${GREEN}提示: XFCE 完全支持 ROS2 Humble、RViz2、Gazebo 等工具${NC}"
        echo ""
        
        # Auto-select XFCE if running non-interactively
        if [ ! -t 0 ]; then
            DESKTOP_CHOICE=1
            echo "非交互模式，自动选择 XFCE"
        else
            read -p "请输入选择 [1/2，默认: 1]: " DESKTOP_CHOICE
            DESKTOP_CHOICE=${DESKTOP_CHOICE:-1}
        fi
        
        case $DESKTOP_CHOICE in
            1)
                echo -e "${GREEN}安装 XFCE 桌面环境...${NC}"
                DEBIAN_FRONTEND=noninteractive apt-get install -y \
                    xfce4 \
                    xfce4-goodies \
                    dbus-x11 \
                    x11-xserver-utils \
                    xfce4-terminal
                DESKTOP_TYPE="xfce"
                echo -e "${GREEN}XFCE 桌面安装完成${NC}"
                ;;
            2)
                echo -e "${GREEN}安装 GNOME 桌面环境...${NC}"
                echo -e "${YELLOW}注意: GNOME 较重，VNC 性能可能不如 XFCE${NC}"
                DEBIAN_FRONTEND=noninteractive apt-get install -y \
                    ubuntu-desktop-minimal \
                    gnome-session \
                    gnome-terminal
                
                # Configure GNOME to use X11 instead of Wayland (required for VNC)
                echo -e "${GREEN}配置 GNOME 使用 X11（VNC 必需）...${NC}"
                if [ -f /etc/gdm3/custom.conf ]; then
                    sed -i 's/#WaylandEnable=false/WaylandEnable=false/' /etc/gdm3/custom.conf
                fi
                
                DESKTOP_TYPE="gnome"
                echo -e "${GREEN}GNOME 桌面安装完成${NC}"
                ;;
            *)
                echo -e "${RED}无效选择，默认安装 XFCE${NC}"
                DEBIAN_FRONTEND=noninteractive apt-get install -y \
                    xfce4 \
                    xfce4-goodies \
                    dbus-x11 \
                    x11-xserver-utils \
                    xfce4-terminal
                DESKTOP_TYPE="xfce"
                echo -e "${GREEN}XFCE 桌面安装完成${NC}"
                ;;
        esac
        
        DESKTOP_INSTALLED=true
    fi
}

# Function to configure firewall
configure_firewall() {
    if command -v ufw &> /dev/null; then
        # Check if ufw is active
        UFW_STATUS=$(ufw status 2>/dev/null | grep -i "Status:" | awk '{print $2}')
        
        if [ "$UFW_STATUS" = "active" ]; then
            echo -e "${GREEN}配置防火墙规则...${NC}"
            ufw allow 5901/tcp comment 'VNC Server'
            ufw allow 6080/tcp comment 'noVNC Web Server'
            echo -e "${GREEN}防火墙规则已配置${NC}"
        else
            echo -e "${YELLOW}防火墙未启动，无需配置规则${NC}"
        fi
    else
        echo "未检测到 ufw 防火墙，跳过"
    fi
}

install_vnc_server() {
    # Install Xvfb and x11vnc
    echo -e "${GREEN}=== 安装 Xvfb 和 x11vnc ===${NC}"
    apt install xvfb x11vnc mesa-utils -y
    echo -e "${GREEN}Xvfb 和 x11vnc 安装完成${NC}"

    # Extract and setup noVNC and websockify
    echo -e "${GREEN}=== 安装 noVNC 和 websockify ===${NC}"
    cd "$SCRIPT_DIR/pkg"
    if [ -f noVNC-1.6.0.tar.gz ]; then
        if [ ! -d noVNC ]; then
            tar -zxf noVNC-1.6.0.tar.gz && mv noVNC-1.6.0 noVNC
            ln -s ${SCRIPT_DIR}/pkg/noVNC/vnc.html ${SCRIPT_DIR}/pkg/noVNC/index.html            
            echo -e "${GREEN}noVNC 解压完成${NC}"
        else
            echo -e "${GREEN}noVNC 已存在，跳过解压${NC}"
        fi
    else
        echo -e "${RED}警告: 找不到 noVNC-1.6.0.tar.gz${NC}"
    fi

    if [ -f websockify-0.13.0.tar.gz ]; then
        if [ ! -d noVNC/utils/websockify ]; then
            tar -zxf websockify-0.13.0.tar.gz 
            mkdir -p noVNC/utils
            mv websockify-0.13.0 noVNC/utils/websockify
            echo -e "${GREEN}websockify 解压完成${NC}"
        else
            echo -e "${GREEN}websockify 已存在，跳过解压${NC}"
        fi
    else
        echo -e "${RED}警告: 找不到 websockify-0.13.0.tar.gz${NC}"
    fi
}


###############################################################################
# 主安装流程
###############################################################################

# Test source speed and switch if necessary
ensure_fast_ubuntu_source
apt-get update

# Check and install desktop environment
check_and_install_desktop

install_vnc_server

# Detect GPU and install VirtualGL if available
detect_gpu
if [ "$HAS_GPU" = true ]; then
    install_virtualgl
else
    VIRTUALGL_INSTALLED=1
fi

# Configure firewall
configure_firewall

exit 0
