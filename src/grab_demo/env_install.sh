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
LIBNCCL_VERSION="2.22.3-1+cuda12.6"
TORCH_WHL="$GRAB_DEMO_DIR/torch-2.7.0-cp310-cp310-linux_aarch64.whl"
TORCH_TXT="$GRAB_DEMO_DIR/torch-2.7.0-cp310-cp310-linux_aarch64.txt"
TORCH_VERSION="2.7.0"
YOLOV8_PT="$YOLO_MODELS_DIR/yolov8n.pt"
YOLOV8_TXT="$GRAB_DEMO_DIR/yolov8n.txt"

PIP_MIRROR="https://mirrors.cloud.tencent.com/pypi/simple"

# ── 工具库 ────────────────────────────────────────────────────────────────────
# shellcheck source=../../lib/apt_mirror_utils.sh
source "$SCRIPT_DIR/../../lib/apt_mirror_utils.sh"

# ── 工具函数 ──────────────────────────────────────────────────────────────────
log_section() { echo; echo "────────────────────────────────────────"; echo "  $1"; echo "────────────────────────────────────────"; }
log_ok()      { echo "  ✓ $1"; }
log_skip()    { echo "  » 已存在，跳过：$1"; }
log_err()     { echo "  ✗ $1" >&2; }
log_info()    { echo "  ℹ $1"; }

is_apt_package_installed() {
    local pkg="$1"
    local status
    status="$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || true)"
    [[ "$status" == "install ok installed" ]]
}

install_apt_packages() {
    local pkgs_to_install=()
    local pkg
    for pkg in "$@"; do
        if is_apt_package_installed "$pkg"; then
            log_skip "apt 包：$pkg"
        else
            pkgs_to_install+=("$pkg")
        fi
    done

    if (( ${#pkgs_to_install[@]} > 0 )); then
        sudo apt install -y "${pkgs_to_install[@]}"
        log_ok "apt 包安装完成：${pkgs_to_install[*]}"
    else
        log_ok "apt 包均已安装，无需安装"
    fi
}

is_pip_spec_satisfied() {
    local spec="$1"
    local pkg_name="$spec"
    local pkg_version=""

    if [[ "$spec" == *"=="* ]]; then
        pkg_name="${spec%%==*}"
        pkg_version="${spec##*==}"
    fi

    python3 - "$pkg_name" "$pkg_version" << 'PY' >/dev/null 2>&1
import sys
from importlib import metadata

name = sys.argv[1]
target_version = sys.argv[2]

try:
    installed_version = metadata.version(name)
except metadata.PackageNotFoundError:
    sys.exit(1)

if target_version and installed_version != target_version:
    sys.exit(2)

sys.exit(0)
PY
}

install_pip_spec_if_needed() {
    local spec="$1"
    shift
    local extra_args=("$@")

    if is_pip_spec_satisfied "$spec"; then
        log_skip "pip 包：$spec"
    else
        pip install -i "$PIP_MIRROR" "${extra_args[@]}" "$spec"
        log_ok "pip 包安装完成：$spec"
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
    install_apt_packages \
        ros-humble-desktop \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-sensor-msgs \
        ros-humble-robot-state-publisher \
        python3-colcon-common-extensions \
        python3-pip
        # python3-opencv \
        # ros-humble-desktop \
        # python3-numpy
    log_ok "ROS 图像依赖安装完成"
}

# 1.5 确保 ROS2 环境已添加到 ~/.bashrc
ensure_ros2_in_bashrc() {
    log_section "步骤 1.5/6：配置 ROS2 环境变量"

    local bashrc="$HOME/.bashrc"
    local ros_setup="/opt/ros/humble/setup.bash"

    if [[ ! -f "$ros_setup" ]]; then
        log_err "ROS2 环境文件不存在：$ros_setup"
        return 1
    fi

    # 检查 ~/.bashrc 中是否已有 source 或 . 命令加载 ROS2 环境
    if grep -qE '^\s*(source|\.)\s+.*ros/humble/setup\.bash' "$bashrc" 2>/dev/null; then
        log_skip "ROS2 环境已配置在 ~/.bashrc"
        return 0
    fi

    # 追加到 ~/.bashrc
    echo "" >> "$bashrc"
    echo "# ROS2 Humble 环境" >> "$bashrc"
    echo "source /opt/ros/humble/setup.bash" >> "$bashrc"
    log_ok "已将 ROS2 环境添加到 ~/.bashrc"

    # 提示用户
    log_info "请在当前终端执行：source ~/.bashrc"
}

# 2. Python 依赖（通过 pip，使用腾讯镜像加速）
install_python_deps() {
    log_section "步骤 2/6：安装 Python 依赖"

    local base_specs=(
        "tqdm==4.66.1"
        "numpy==1.23.5"
        "pandas==1.3.5"
        "seaborn==0.13.0"
        "thop"
        "py-cpuinfo==9.0.0"
        "opencv-python==4.11.0.86"
    )
    local spec
    for spec in "${base_specs[@]}"; do
        install_pip_spec_if_needed "$spec"
    done

    # ultralytics 不带依赖安装，避免覆盖 arm64 专用 torch
    install_pip_spec_if_needed "ultralytics==8.3.232" --no-deps

    # arm64: torchvision 单独安装（不带依赖，避免覆盖 arm64 专用 torch）
    # x86_64: torchvision 随 torch 一起安装，此处跳过
    local arch
    arch="$(uname -m)"
    if [[ "$arch" == "aarch64" ]]; then
        install_pip_spec_if_needed "torchvision==0.22.0" --no-deps
    fi

    log_ok "Python 依赖安装完成"
}

# 3. libnccl2（torch 运行时依赖的 NCCL 动态库，仅 arm64 需要）
install_libnccl2() {
    local arch
    arch="$(uname -m)"
    if [[ "$arch" != "aarch64" ]]; then
        log_section "步骤 3/6：安装 libnccl2"
        log_skip "libnccl2（仅 arm64 需要，当前架构：$arch）"
        return 0
    fi

    log_section "步骤 3/6：安装 libnccl2"
    download_if_missing "$LIBNCCL_DEB" "$LIBNCCL_TXT" "libnccl2 deb 包"

    local installed_libnccl_version
    installed_libnccl_version="$(dpkg-query -W -f='${Version}' libnccl2 2>/dev/null || true)"
    if [[ "$installed_libnccl_version" == "$LIBNCCL_VERSION" ]]; then
        log_skip "libnccl2（版本 $LIBNCCL_VERSION）"
    else
        sudo dpkg -i "$LIBNCCL_DEB"
        log_ok "libnccl2 安装完成"
    fi
    sudo ldconfig

    echo "  验证 libnccl.so.2："
    local ldconfig_output
    ldconfig_output="$(ldconfig -p 2>/dev/null || true)"
    if grep -Fq "libnccl.so.2" <<< "$ldconfig_output"; then
        grep -F "libnccl.so.2" <<< "$ldconfig_output"
    else
        echo "  ⚠ 未找到 libnccl.so.2，请检查安装"
    fi
}

# 4. torch（根据架构自动选择安装方式）
install_torch() {
    local arch
    arch="$(uname -m)"

    case "$arch" in
        aarch64)
            log_section "步骤 4/6：安装 torch（arm64 GPU 版）"
            download_if_missing "$TORCH_WHL" "$TORCH_TXT" "torch whl 包"

            local installed_torch_version
            installed_torch_version="$(python3 - << 'PY'
try:
    import torch
    print(torch.__version__)
except Exception:
    pass
PY
)"
            if [[ "$installed_torch_version" == "$TORCH_VERSION" ]]; then
                log_skip "torch（版本 $TORCH_VERSION）"
            else
                pip install "$TORCH_WHL"
                log_ok "torch 安装完成"
            fi
            ;;
        x86_64)
            log_section "步骤 4/6：安装 torch（x86_64 GPU 版）"
            # 检查是否已安装支持 CUDA 的 torch
            local has_cuda_torch
            has_cuda_torch="$(python3 - << 'PY'
try:
    import torch
    print("cuda" if torch.cuda.is_available() else "no_cuda")
except Exception:
    print("none")
PY
)"
            if [[ "$has_cuda_torch" == "cuda" ]]; then
                log_skip "torch（已安装 CUDA 版本：$(python3 -c 'import torch; print(torch.__version__)' 2>/dev/null || echo '未知')）"
            else
                log_info "正在安装 torch CUDA 版本..."
                pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124
                log_ok "torch 安装完成"
            fi
            ;;
        *)
            log_err "不支持的架构：$arch"
            exit 1
            ;;
    esac
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
    install_apt_packages \
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

    log_section "apt 源速度检测"
    ensure_fast_ubuntu_source
    ensure_fast_ros2_source
    sudo apt update
    install_ros_image_packages
    ensure_ros2_in_bashrc
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
