#!/usr/bin/env bash
# =============================================================================
# env_install.sh
# 天工抓取示例 · 环境安装脚本
# 运行环境：Orin 板 (Ubuntu 22.04 arm64, ROS2 Humble)
# 用法：bash src/grab_demo/env_install.sh
# =============================================================================

set -euo pipefail

# ── 路径常量 ──────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
GRAB_DEMO_DIR="$SCRIPT_DIR"
YOLO_MODELS_DIR="$PROJECT_ROOT/yolo_models"

LIBNCCL_DEB="$GRAB_DEMO_DIR/libnccl2_2.22.3-1+cuda12.6_arm64.deb"
LIBNCCL_TXT="$GRAB_DEMO_DIR/libnccl2_2.22.3-1+cuda12.6_arm64.txt"
TORCH_WHL="$GRAB_DEMO_DIR/torch-2.7.0-cp310-cp310-linux_aarch64.whl"
TORCH_TXT="$GRAB_DEMO_DIR/torch-2.7.0-cp310-cp310-linux_aarch64.txt"
YOLOV8_PT="$YOLO_MODELS_DIR/yolov8n.pt"
YOLOV8_TXT="$GRAB_DEMO_DIR/yolov8n.txt"

# apt 源相关路径
SOURCES_LIST="/etc/apt/sources.list"
SOURCES_BACKUP=""          # 备份文件路径，切换源后设置
SOURCES_FAST_MIRROR="$PROJECT_ROOT/vnc/sources.list.arm64"
# 检测 apt 源速度时使用的测试 URL（ubuntu-ports 小文件，约 2KB）
APT_SPEED_TEST_URL="http://ports.ubuntu.com/ubuntu-ports/dists/jammy/Release.gpg"
APT_SPEED_TIMEOUT=8        # 超过该秒数视为源过慢（秒）

PIP_MIRROR="https://mirrors.cloud.tencent.com/pypi/simple"

# ── 工具函数 ──────────────────────────────────────────────────────────────────
log_section() { echo; echo "────────────────────────────────────────"; echo "  $1"; echo "────────────────────────────────────────"; }
log_ok()      { echo "  ✓ $1"; }
log_skip()    { echo "  » 已存在，跳过：$1"; }
log_err()     { echo "  ✗ $1" >&2; }
log_info()    { echo "  ℹ $1"; }

# ── apt 源速度检测与自动切换 ──────────────────────────────────────

# 脱离常量，用于记录是否已切换源（不能放在子层 shell 里）
SOURCES_SWITCHED=false

# 如果切换了源，trap EXIT 时自动还原
restore_apt_sources() {
    if [[ "$SOURCES_SWITCHED" == true && -n "$SOURCES_BACKUP" && -f "$SOURCES_BACKUP" ]]; then
        echo
        log_section "还原 apt 源"
        sudo cp "$SOURCES_BACKUP" "$SOURCES_LIST"
        sudo apt update -qq
        sudo rm -f "$SOURCES_BACKUP"
        log_ok "已还原至原安装源：$SOURCES_LIST"
    fi
}
trap restore_apt_sources EXIT

# 测试当前 apt 源速度，返回 0 表示速度合格，1 表示过慢
check_apt_speed() {
    log_info "测试 apt 源速度（超时阈値 ${APT_SPEED_TIMEOUT}s）..."
    if curl -fsSL --max-time "$APT_SPEED_TIMEOUT" \
             -o /dev/null "$APT_SPEED_TEST_URL" 2>/dev/null; then
        log_ok "apt 源速度正常"
        return 0
    else
        log_info "apt 源响应超过 ${APT_SPEED_TIMEOUT}s，视为過慢"
        return 1
    fi
}

# 备份当前源并切换到清华镜像
switch_to_fast_mirror() {
    if [[ ! -f "$SOURCES_FAST_MIRROR" ]]; then
        log_err "备用镜像文件不存在：$SOURCES_FAST_MIRROR"
        log_info "继续使用现有源，如安装失败请手动更换镜像"
        return
    fi
    SOURCES_BACKUP="${SOURCES_LIST}.$(date +%Y%m%d_%H%M%S).bak"
    log_info "备份现有源到：$SOURCES_BACKUP"
    sudo cp "$SOURCES_LIST" "$SOURCES_BACKUP"
    sudo cp "$SOURCES_FAST_MIRROR" "$SOURCES_LIST"
    SOURCES_SWITCHED=true
    log_ok "已切换到清华镜像（脚本退出后将自动还原）"
}

# 在所有 apt 操作前调用一次：检测速度，必要时自动切换
ensure_fast_apt_sources() {
    log_section "apt 源速度检测"
    if ! check_apt_speed; then
        switch_to_fast_mirror
        sudo apt update -qq
    fi
}

# 从 .txt 文件读取下载地址，若目标文件不存在则下载
download_if_missing() {
    local file="$1"   # 目标文件路径
    local txt="$2"    # 存放下载地址的 .txt 文件路径
    local desc="$3"   # 用于日志的描述

    if [[ -f "$file" ]]; then
        log_skip "$desc"
        return 0
    fi

    if [[ ! -f "$txt" ]]; then
        log_err "下载地址文件不存在：$txt"
        exit 1
    fi

    local url
    url="$(head -n1 "$txt" | tr -d '[:space:]')"
    if [[ -z "$url" ]]; then
        log_err "下载地址文件为空：$txt"
        exit 1
    fi

    echo "  ↓ 下载 $desc ..."
    wget -q --show-progress -O "$file" "$url"
    log_ok "$desc 下载完成"
}

# ── 安装步骤（每个步骤封装为独立函数）────────────────────────────────────────

# 1. ROS 图像相关软件包
install_ros_image_packages() {
    log_section "步骤 1/6：安装 ROS 图像相关依赖"
    sudo apt update -qq
    sudo apt install -y \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-sensor-msgs \
        python3-opencv \
        python3-numpy
    log_ok "ROS 图像依赖安装完成"
}

# 2. Python 依赖（通过 pip，使用腾讯镜像加速）
install_python_deps() {
    log_section "步骤 2/6：安装 Python 依赖"

    # 基础依赖
    pip install -i "$PIP_MIRROR" \
        tqdm==4.66.1 \
        numpy==1.23.5 \
        pandas==1.3.5 \
        seaborn==0.13.0 \
        thop \
        py-cpuinfo==9.0.0 \
        opencv-python==4.11.0.86

    # ultralytics 和 torchvision 不带依赖安装，避免覆盖 arm64 专用 torch
    pip install -i "$PIP_MIRROR" --no-deps \
        ultralytics==8.3.232 \
        torchvision==0.22.0

    log_ok "Python 依赖安装完成"
}

# 3. libnccl2（torch 运行时依赖的 NCCL 动态库）
install_libnccl2() {
    log_section "步骤 3/6：安装 libnccl2"
    download_if_missing "$LIBNCCL_DEB" "$LIBNCCL_TXT" "libnccl2 deb 包"

    sudo dpkg -i "$LIBNCCL_DEB"
    sudo ldconfig
    log_ok "libnccl2 安装完成"

    echo "  验证 libnccl.so.2："
    if ldconfig -p | grep -q "libnccl.so.2"; then
        ldconfig -p | grep "libnccl.so.2"
    else
        echo "  ⚠ 未找到 libnccl.so.2，请检查安装"
    fi
}

# 4. torch（arm64 GPU 版，从本地 whl 安装）
install_torch() {
    log_section "步骤 4/6：安装 torch（arm64 GPU 版）"
    download_if_missing "$TORCH_WHL" "$TORCH_TXT" "torch whl 包"
    pip install "$TORCH_WHL"
    log_ok "torch 安装完成"
}

# 5. YOLO 预训练模型
ensure_yolo_model() {
    log_section "步骤 5/6：准备 YOLO 预训练模型"
    mkdir -p "$YOLO_MODELS_DIR"
    download_if_missing "$YOLOV8_PT" "$YOLOV8_TXT" "yolov8n.pt"
    log_ok "YOLO 模型已就绪：$YOLOV8_PT"
}

# 6. MoveIt2
install_moveit() {
    log_section "步骤 6/6：安装 MoveIt2"
    sudo apt install -y \
        ros-humble-moveit \
        ros-humble-moveit-ros-visualization
    log_ok "MoveIt2 安装完成"
}

# ── 安装结果验证 ──────────────────────────────────────────────────────────────
verify_installation() {
    log_section "验证安装结果"
    python3 - << 'EOF'
import sys
try:
    import torch
    import ultralytics
    import numpy as np
    print(f"  torch       : {torch.__version__}")
    print(f"  ultralytics : {ultralytics.__version__}")
    print(f"  numpy       : {np.__version__}")
    print(f"  CUDA 可用    : {torch.cuda.is_available()}")
    print(f"  编译 CUDA    : {torch.version.cuda}")
    print(f"  cuDNN        : {torch.backends.cudnn.version()}")
except ImportError as e:
    print(f"  ✗ 导入失败：{e}", file=sys.stderr)
    sys.exit(1)
EOF
}

# ── 主流程 ────────────────────────────────────────────────────────────────────
main() {
    echo "════════════════════════════════════════════"
    echo "  天工抓取示例 · 环境安装脚本"
    echo "  项目根目录：$PROJECT_ROOT"
    echo "════════════════════════════════════════════"

    ensure_fast_apt_sources
    install_ros_image_packages
    install_python_deps
    install_libnccl2
    install_torch
    ensure_yolo_model
    install_moveit
    verify_installation

    echo
    echo "════════════════════════════════════════════"
    echo "  ✓ 全部安装步骤已完成"
    echo "════════════════════════════════════════════"
    # restore_apt_sources 由 trap EXIT 自动触发，无需手动调用
}

main "$@"
