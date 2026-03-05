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

# apt 源相关路径
SOURCES_LIST="/etc/apt/sources.list"
SOURCES_BACKUP=""          # 备份文件路径，切换源后设置
SOURCES_FAST_MIRROR="$PROJECT_ROOT/vnc/sources.list.arm64"
# ROS2 apt 源（单独的 sources.list.d 文件）
ROS2_SOURCES_LIST=""        # 运行时自动检测
ROS2_SOURCES_BACKUP=""      # 切换后设置
ROS2_SOURCES_LINK_TARGET="" # 若原文件是符号链接，记录原链接目标，还原时重建
APT_SPEED_MIN_BPMS=1048576  # apt 源下载速度低于该小时就换源（1 MB/s）

PIP_MIRROR="https://mirrors.cloud.tencent.com/pypi/simple"

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

# ── apt 源速度检测与自动切换 ──────────────────────────────────────

# 脱离常量，用于记录是否已切换源（不能放在子层 shell 里）
SOURCES_SWITCHED=false

# 如果切换了源，trap EXIT 时自动还原
restore_apt_sources() {
    if [[ "$SOURCES_SWITCHED" == true && -n "$SOURCES_BACKUP" && -f "$SOURCES_BACKUP" ]]; then
        echo
        log_section "还原 apt 源"
        sudo cp "$SOURCES_BACKUP" "$SOURCES_LIST"
        sudo rm -f "$SOURCES_BACKUP"
        log_ok "已还原 Ubuntu 源：$SOURCES_LIST"
    fi
    if [[ -n "$ROS2_SOURCES_LINK_TARGET" && -n "$ROS2_SOURCES_LIST" ]]; then
        # 原文件是符号链接：删除当前普通文件，重建符号链接
        sudo rm -f "$ROS2_SOURCES_LIST"
        sudo ln -s "$ROS2_SOURCES_LINK_TARGET" "$ROS2_SOURCES_LIST"
        log_ok "已还原 ROS2 源（重建符号链接 $ROS2_SOURCES_LIST → $ROS2_SOURCES_LINK_TARGET）"
    elif [[ -n "$ROS2_SOURCES_BACKUP" && -f "$ROS2_SOURCES_BACKUP" && -n "$ROS2_SOURCES_LIST" ]]; then
        # 原文件是普通文件：直接覆盖还原内容
        sudo cp "$ROS2_SOURCES_BACKUP" "$ROS2_SOURCES_LIST"
        sudo rm -f "$ROS2_SOURCES_BACKUP"
        log_ok "已还原 ROS2 源：$ROS2_SOURCES_LIST"
    fi
}
trap restore_apt_sources EXIT

# 测试当前 apt 源速度，返回 0 表示速度合格，1 表示过慢
# 做法：从 sources.list 动态读取镜像地址，下载 ~2KB 的 Release.gpg 小文件测速
check_apt_speed() {
    local mirror_url
    mirror_url=$(grep -m1 '^deb ' "$SOURCES_LIST" 2>/dev/null | awk '{print $2}')
    if [[ -z "$mirror_url" ]]; then
        log_info "无法解析 apt 源地址，跳过速度检测"
        return 0
    fi
    local test_url="${mirror_url%/}/dists/jammy/Release.gpg"
    log_info "检测 apt 源速度（下载 Release.gpg，低于 ${APT_SPEED_MIN_BPMS} B/s 就换源）..."
    log_info "测试地址：$test_url"
    local speed
    speed=$(curl -fsSL --max-time 15 \
        --write-out "%{speed_download}" \
        -o /dev/null "$test_url" 2>/dev/null || echo "0")
    # curl 输出的是浮点数（如 524288.000），取整数部分
    speed=${speed%%.*}
    log_info "测得速度：${speed} B/s（阈值：${APT_SPEED_MIN_BPMS} B/s）"
    if [[ ${speed} -lt ${APT_SPEED_MIN_BPMS} ]]; then
        return 1  # 过慢
    else
        log_ok "apt 源速度正常（$(( speed / 1024 )) KB/s）"
        return 0
    fi
}

# 生成 ROS2 镜像源文件：仅替换 URI，保留原始签名配置（包括内嵌公钥）
generate_ros2_mirror_sources() {
    local src_file="$1"
    local dst_file="$2"

    if grep -q '^URIs:' "$src_file"; then
        sed 's#^URIs: .*#URIs: https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#; s#^Types: .*#Types: deb#' "$src_file" > "$dst_file"
    else
        sed 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g; s#https://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g; /^deb-src[[:space:]].*ros2\/ubuntu/d' "$src_file" > "$dst_file"
    fi
}

# 备份当前源并切换到清华镜像
switch_to_fast_mirror() {
    # 切换 Ubuntu 源
    if [[ ! -f "$SOURCES_FAST_MIRROR" ]]; then
        log_err "备用 Ubuntu 镜像文件不存在：$SOURCES_FAST_MIRROR"
        log_info "继续使用现有源，如安装失败请手动更换镜像"
    else
        SOURCES_BACKUP="${SOURCES_LIST}.$(date +%Y%m%d_%H%M%S).bak"
        log_info "备份现有 Ubuntu 源到：$SOURCES_BACKUP"
        sudo cp "$SOURCES_LIST" "$SOURCES_BACKUP"
        sudo cp "$SOURCES_FAST_MIRROR" "$SOURCES_LIST"
        SOURCES_SWITCHED=true
        log_ok "已切换 Ubuntu 源到清华镜像"
    fi
    # 切换 ROS2 源（仅替换 URI，保留当前签名配置）
    for _f in /etc/apt/sources.list.d/ros2.list \
              /etc/apt/sources.list.d/ros2-latest.list \
              /etc/apt/sources.list.d/ros2.sources; do
        if [[ -f "$_f" ]]; then
            ROS2_SOURCES_LIST="$_f"
            break
        fi
    done
    if [[ -z "$ROS2_SOURCES_LIST" ]]; then
        log_info "未检测到 ROS2 apt 源文件，跳过 ROS2 源切换"
    else
        local ros2_src_for_render="$ROS2_SOURCES_LIST"
        local ros2_rendered
        ros2_rendered="$(mktemp)"

        # 若为符号链接：记录链接目标；渲染时读取链接目标内容，确保拿到最新官方签名配置
        if [[ -L "$ROS2_SOURCES_LIST" ]]; then
            ROS2_SOURCES_LINK_TARGET=$(readlink "$ROS2_SOURCES_LIST")
            log_info "ROS2 源是符号链接，链接目标：$ROS2_SOURCES_LINK_TARGET"
            if [[ -f "$ROS2_SOURCES_LINK_TARGET" ]]; then
                ros2_src_for_render="$ROS2_SOURCES_LINK_TARGET"
            fi
        else
            ROS2_SOURCES_BACKUP="${ROS2_SOURCES_LIST}.$(date +%Y%m%d_%H%M%S).bak"
            log_info "备份现有 ROS2 源到：$ROS2_SOURCES_BACKUP"
            sudo cp "$ROS2_SOURCES_LIST" "$ROS2_SOURCES_BACKUP"
        fi

        generate_ros2_mirror_sources "$ros2_src_for_render" "$ros2_rendered"

        if [[ -n "$ROS2_SOURCES_LINK_TARGET" ]]; then
            sudo rm -f "$ROS2_SOURCES_LIST"
            sudo cp "$ros2_rendered" "$ROS2_SOURCES_LIST"
            log_ok "已切换 ROS2 源到清华镜像（保留官方签名配置，已解除符号链接）"
        else
            sudo cp "$ros2_rendered" "$ROS2_SOURCES_LIST"
            log_ok "已切换 ROS2 源到清华镜像（保留官方签名配置）"
        fi
        rm -f "$ros2_rendered"
    fi
}

# 在所有 apt 操作前调用一次：检测速度，必要时自动切换
ensure_fast_apt_sources() {
    log_section "apt 源速度检测"
    if ! check_apt_speed; then
        switch_to_fast_mirror
    fi
    sudo apt update
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
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-sensor-msgs \
        python3-pip
        # python3-opencv \
        # python3-numpy
    log_ok "ROS 图像依赖安装完成"
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

    # ultralytics 和 torchvision 不带依赖安装，避免覆盖 arm64 专用 torch
    install_pip_spec_if_needed "ultralytics==8.3.232" --no-deps
    install_pip_spec_if_needed "torchvision==0.22.0" --no-deps

    log_ok "Python 依赖安装完成"
}

# 3. libnccl2（torch 运行时依赖的 NCCL 动态库）
install_libnccl2() {
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

# 4. torch（arm64 GPU 版，从本地 whl 安装）
install_torch() {
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
