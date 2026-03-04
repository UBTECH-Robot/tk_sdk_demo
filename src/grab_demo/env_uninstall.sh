#!/usr/bin/env bash
# =============================================================================
# env_uninstall.sh
# 天工抓取示例 · 环境卸载脚本
# 仅卸载由 env_install.sh 显式安装的包和库，不影响其他已有环境。
# 用法：bash src/grab_demo/env_uninstall.sh
# =============================================================================

set -euo pipefail

# ── 工具函数 ──────────────────────────────────────────────────────────────────
log_section() { echo; echo "────────────────────────────────────────"; echo "  $1"; echo "────────────────────────────────────────"; }
log_ok()      { echo "  ✓ $1"; }
log_skip()    { echo "  » 未安装，跳过：$1"; }

# 卸载 apt 包（仅当已安装时操作）
apt_remove() {
    local pkg="$1"
    if dpkg -s "$pkg" &>/dev/null; then
        sudo apt remove -y "$pkg"
        log_ok "已卸载：$pkg"
    else
        log_skip "$pkg"
    fi
}

# 卸载 pip 包（仅当已安装时操作）
pip_uninstall() {
    local pkg="$1"
    if pip show "$pkg" &>/dev/null; then
        pip uninstall -y "$pkg"
        log_ok "已卸载：$pkg"
    else
        log_skip "$pkg"
    fi
}

# ── 卸载步骤（与 env_install.sh 顺序一一对应）───────────────────────────────

# 6. MoveIt2（后装先卸）
uninstall_moveit() {
    log_section "步骤 1/5：卸载 MoveIt2"
    apt_remove ros-humble-moveit-ros-visualization
    apt_remove ros-humble-moveit
    log_ok "MoveIt2 卸载完成"
}

# 4. torch
uninstall_torch() {
    log_section "步骤 2/5：卸载 torch"
    pip_uninstall torch
    log_ok "torch 卸载完成"
}

# 3. libnccl2
uninstall_libnccl2() {
    log_section "步骤 3/5：卸载 libnccl2"
    if dpkg -s libnccl2 &>/dev/null; then
        sudo dpkg --remove libnccl2
        sudo ldconfig
        log_ok "libnccl2 卸载完成"
    else
        log_skip "libnccl2"
    fi
}

# 2. Python 依赖
uninstall_python_deps() {
    log_section "步骤 4/5：卸载 Python 依赖"

    # 与 env_install.sh 中 pip install --no-deps 对应
    pip_uninstall ultralytics
    pip_uninstall torchvision

    # 与 env_install.sh 中基础依赖对应
    pip_uninstall opencv-python
    pip_uninstall py-cpuinfo
    # pip_uninstall thop
    # pip_uninstall seaborn
    # pip_uninstall pandas
    # pip_uninstall numpy
    # pip_uninstall tqdm

    log_ok "Python 依赖卸载完成"
}

# 1. ROS 图像相关软件包
uninstall_ros_image_packages() {
    log_section "步骤 5/5：卸载 ROS 图像相关依赖"
    # apt_remove python3-numpy
    # apt_remove python3-opencv
    apt_remove ros-humble-sensor-msgs
    apt_remove ros-humble-image-transport
    apt_remove ros-humble-cv-bridge
    echo "  ℹ 如需清理孤立依赖，请手动执行： sudo apt autoremove"
    log_ok "ROS 图像依赖卸载完成"
}

# ── 主流程 ────────────────────────────────────────────────────────────────────
main() {
    echo "════════════════════════════════════════════"
    echo "  天工抓取示例 · 环境卸载脚本"
    echo "  仅卸载 env_install.sh 显式安装的内容"
    echo "════════════════════════════════════════════"
    echo
    echo "  注意：以下内容不会被此脚本删除"
    echo "    · 下载的 .deb / .whl / .pt 源文件（请自行管理）"
    echo "    · env_install.sh 安装的包所拉取的间接依赖"
    echo "    · ROS2 Humble 基础框架本身"
    echo

    read -r -p "  确认继续卸载？[y/N] " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        echo "  已取消。"
        exit 0
    fi

    uninstall_moveit
    uninstall_torch
    uninstall_libnccl2
    uninstall_python_deps
    uninstall_ros_image_packages

    echo
    echo "════════════════════════════════════════════"
    echo "  ✓ 卸载完成"
    echo "════════════════════════════════════════════"
}

main "$@"
