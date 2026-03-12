#!/usr/bin/env bash
# =============================================================================
# lib/apt_mirror_utils.sh
# apt 源速度检测与自动切换工具库
#
# 用法：source "$(dirname "${BASH_SOURCE[0]}")/../../lib/apt_mirror_utils.sh"
#
# 公开入口：
#   ensure_fast_ubuntu_source   检测 Ubuntu 源，慢则切换（不含 apt update）
#   ensure_fast_ros2_source     检测 ROS2 源，慢则切换（不含 apt update）
#
# 可选：在 source 前设置以下变量覆盖默认值
#   APT_SPEED_MIN_BPMS          换源阈值（字节/秒），默认 1048576 (1MB/s)
#   APT_SOURCES_FAST_MIRROR     Ubuntu 镜像源文件路径（覆盖按架构自动选择）
# =============================================================================

# ── 全局状态变量 ──────────────────────────────────────────────────────────────
SOURCES_LIST="/etc/apt/sources.list"
SOURCES_BACKUP=""
ROS2_SOURCES_LIST=""
ROS2_SOURCES_BACKUP=""
ROS2_SOURCES_LINK_TARGET=""
SOURCES_SWITCHED=false

: "${APT_SPEED_MIN_BPMS:=1048576}"

# lib 自身所在目录（用于定位同项目资源文件）
_APT_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 日志函数（若调用方已定义则不覆盖）────────────────────────────────────────
declare -f log_ok   >/dev/null 2>&1 || log_ok()   { echo "  ✓ $1"; }
declare -f log_info >/dev/null 2>&1 || log_info() { echo "  ℹ $1"; }
declare -f log_err  >/dev/null 2>&1 || log_err()  { echo "  ✗ $1" >&2; }

# ── Ubuntu 镜像源文件路径（按架构自动选择，可被 APT_SOURCES_FAST_MIRROR 覆盖）──
_get_ubuntu_fast_mirror() {
    if [[ -n "${APT_SOURCES_FAST_MIRROR:-}" ]]; then
        echo "$APT_SOURCES_FAST_MIRROR"
        return
    fi
    local arch
    arch=$(uname -m)
    if [[ "$arch" == "x86_64" ]]; then
        echo "$_APT_LIB_DIR/sources.list.x86_64"
    else
        echo "$_APT_LIB_DIR/sources.list.arm64"
    fi
}

# ── 还原函数 + EXIT trap ──────────────────────────────────────────────────────
restore_apt_sources() {
    if [[ "$SOURCES_SWITCHED" == true && -n "$SOURCES_BACKUP" && -f "$SOURCES_BACKUP" ]]; then
        sudo cp "$SOURCES_BACKUP" "$SOURCES_LIST"
        sudo rm -f "$SOURCES_BACKUP"
        log_ok "已还原 Ubuntu 源：$SOURCES_LIST"
    fi
    if [[ -n "$ROS2_SOURCES_LINK_TARGET" && -n "$ROS2_SOURCES_LIST" ]]; then
        sudo rm -f "$ROS2_SOURCES_LIST"
        sudo ln -s "$ROS2_SOURCES_LINK_TARGET" "$ROS2_SOURCES_LIST"
        log_ok "已还原 ROS2 源（重建符号链接 $ROS2_SOURCES_LIST → $ROS2_SOURCES_LINK_TARGET）"
    elif [[ -n "$ROS2_SOURCES_BACKUP" && -f "$ROS2_SOURCES_BACKUP" && -n "$ROS2_SOURCES_LIST" ]]; then
        sudo cp "$ROS2_SOURCES_BACKUP" "$ROS2_SOURCES_LIST"
        sudo rm -f "$ROS2_SOURCES_BACKUP"
        log_ok "已还原 ROS2 源：$ROS2_SOURCES_LIST"
    fi
}
trap restore_apt_sources EXIT

# ── 底层测速 ──────────────────────────────────────────────────────────────────
# $1=mirror_url  $2=suite  $3=skip_suite_updates（非空则跳过 jammy-updates 候选）
# 返回 0=合格，1=过慢
_check_mirror_speed() {
    local mirror_url="$1"
    local suite="$2"
    local skip_suite_updates="${3:-}"

    # 按 deb 架构选择 binary-xxx 路径（Packages.xz 是架构相关的）
    local deb_arch
    deb_arch=$(dpkg --print-architecture 2>/dev/null || echo "arm64")

    local candidates=()
    [[ -z "$skip_suite_updates" ]] && \
        candidates+=("${mirror_url%/}/dists/${suite}-updates/main/binary-${deb_arch}/Packages.xz")
    candidates+=(
        "${mirror_url%/}/dists/${suite}/main/binary-${deb_arch}/Packages.xz"
        "${mirror_url%/}/dists/${suite}/Release"
    )

    local test_url=""
    local candidate
    for candidate in "${candidates[@]}"; do
        if curl -fsI --max-time 8 "$candidate" >/dev/null 2>&1; then
            test_url="$candidate"
            break
        fi
    done

    log_info "测试地址：$test_url"

    if [[ -z "$test_url" ]]; then
        log_info "无法定位测速文件，跳过速度检测"
        return 0
    fi

    log_info "检测 apt 源速度（基于索引文件分片下载，中位数低于 ${APT_SPEED_MIN_BPMS} B/s 就换源）..."

    local samples=()
    local raw s i
    for i in 1 2 3; do
        raw=$(curl -fsSL --max-time 15 --range 0-1048575 \
            --write-out "%{speed_download}" \
            -o /dev/null "$test_url" 2>/dev/null || echo "0")
        s=${raw%%.*}
        [[ -z "$s" ]] && s=0
        samples+=("$s")
    done

    IFS=$'\n' read -r -d '' -a samples_sorted < <(printf '%s\n' "${samples[@]}" | sort -n && printf '\0')
    local speed="${samples_sorted[1]}"

    log_info "测速样本：${samples[0]} / ${samples[1]} / ${samples[2]} B/s"
    log_info "测得中位数：${speed} B/s（阈值：${APT_SPEED_MIN_BPMS} B/s）"
    if [[ ${speed} -lt ${APT_SPEED_MIN_BPMS} ]]; then
        return 1  # 过慢
    else
        log_ok "apt 源速度正常（$(( speed / 1024 )) KB/s）"
        return 0
    fi
}

# ── 测速包装 ──────────────────────────────────────────────────────────────────
# 测试 Ubuntu apt 源速度，返回 0=合格，1=过慢
check_apt_speed() {
    local deb_line
    deb_line=$(grep -m1 '^deb ' "$SOURCES_LIST" 2>/dev/null || true)
    local mirror_url suite
    # 直接匹配 http/https URL，不受 [options] 影响
    mirror_url=$(grep -oE 'https?://[^[:space:]]+' <<<"$deb_line" | head -1)
    # suite 是 URL 后的第一个字段
    suite=$(awk '{
        for(i=1;i<=NF;i++) {
            if($i ~ /^https?:\/\//) { print $(i+1); exit }
        }
    }' <<<"$deb_line")
    if [[ -z "$mirror_url" ]]; then
        log_info "无法解析 Ubuntu 源地址，跳过速度检测"
        return 0
    fi
    [[ -z "$suite" ]] && suite="jammy"
    _check_mirror_speed "$mirror_url" "$suite"
}

# 测试 ROS2 apt 源速度，返回 0=合格，1=过慢
# ROS2 镜像只有 jammy/，没有 jammy-updates/，传 skip_suite_updates 避免无效请求
check_ros2_apt_speed() {
    local ros2_src=""
    local _f
    for _f in /etc/apt/sources.list.d/ros2.list \
              /etc/apt/sources.list.d/ros2-latest.list \
              /etc/apt/sources.list.d/ros2.sources; do
        [[ -f "$_f" ]] && { ros2_src="$_f"; break; }
    done
    if [[ -z "$ros2_src" ]]; then
        log_info "未检测到 ROS2 apt 源文件，跳过 ROS2 源速度检测"
        return 0
    fi
    # 兼容 DEB822（URIs:）和旧格式（deb ...）
    local mirror_url
    if grep -q '^URIs:' "$ros2_src" 2>/dev/null; then
        # DEB822 格式
        mirror_url=$(awk '/^URIs:/{print $2; exit}' "$ros2_src")
    else
        # one-line 格式：直接匹配 http/https URL，不受 [options] 影响
        mirror_url=$(grep -m1 '^deb ' "$ros2_src" | grep -oE 'https?://[^[:space:]]+' | head -1)
    fi
    if [[ -z "$mirror_url" ]]; then
        log_info "无法解析 ROS2 源地址，跳过速度检测"
        return 0
    fi
    _check_mirror_speed "$mirror_url" "jammy" skip_suite_updates
}

# ── 切换函数 ──────────────────────────────────────────────────────────────────
# 将 Ubuntu apt 源切换到清华镜像
switch_ubuntu_to_fast_mirror() {
    local fast_mirror
    fast_mirror="$(_get_ubuntu_fast_mirror)"
    if [[ ! -f "$fast_mirror" ]]; then
        log_err "备用 Ubuntu 镜像文件不存在：$fast_mirror"
        log_info "继续使用现有源，如安装失败请手动更换镜像"
        return 0
    fi
    SOURCES_BACKUP="${SOURCES_LIST}.$(date +%Y%m%d_%H%M%S).bak"
    log_info "备份现有 Ubuntu 源到：$SOURCES_BACKUP"
    sudo cp "$SOURCES_LIST" "$SOURCES_BACKUP"
    sudo cp "$fast_mirror" "$SOURCES_LIST"
    SOURCES_SWITCHED=true
    log_ok "已切换 Ubuntu 源到清华镜像"
}

# 生成 ROS2 镜像源文件：仅替换 URI，保留原始签名配置（包括内嵌公钥）
generate_ros2_mirror_sources() {
    local src_file="$1"
    local dst_file="$2"
    if grep -q '^URIs:' "$src_file"; then
        sed 's#^URIs: .*#URIs: https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#
             s#^Types: .*#Types: deb#' "$src_file" > "$dst_file"
    else
        sed 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g
             s#https://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g
             /^deb-src[[:space:]].*ros2\/ubuntu/d' "$src_file" > "$dst_file"
    fi
}

# 将 ROS2 apt 源切换到清华镜像（仅替换 URI，保留当前签名配置）
switch_ros2_to_fast_mirror() {
    local _f
    for _f in /etc/apt/sources.list.d/ros2.list \
              /etc/apt/sources.list.d/ros2-latest.list \
              /etc/apt/sources.list.d/ros2.sources; do
        [[ -f "$_f" ]] && { ROS2_SOURCES_LIST="$_f"; break; }
    done
    if [[ -z "$ROS2_SOURCES_LIST" ]]; then
        log_info "未检测到 ROS2 apt 源文件，跳过 ROS2 源切换"
        return 0
    fi

    local ros2_src_for_render="$ROS2_SOURCES_LIST"
    local ros2_rendered
    ros2_rendered="$(mktemp)"

    # 若为符号链接：记录链接目标；渲染时读取链接目标内容，确保拿到最新官方签名配置
    if [[ -L "$ROS2_SOURCES_LIST" ]]; then
        ROS2_SOURCES_LINK_TARGET=$(readlink "$ROS2_SOURCES_LIST")
        log_info "ROS2 源是符号链接，链接目标：$ROS2_SOURCES_LINK_TARGET"
        [[ -f "$ROS2_SOURCES_LINK_TARGET" ]] && ros2_src_for_render="$ROS2_SOURCES_LINK_TARGET"
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
}

# ── 公开入口 ──────────────────────────────────────────────────────────────────
# 检测 Ubuntu 源，慢则切换（不含 apt update，由调用方决定时机）
ensure_fast_ubuntu_source() {
    log_info "检测 Ubuntu 源..."
    if ! check_apt_speed; then
        switch_ubuntu_to_fast_mirror
    fi
}

# 检测 ROS2 源，慢则切换（不含 apt update，由调用方决定时机）
ensure_fast_ros2_source() {
    log_info "检测 ROS2 源..."
    if ! check_ros2_apt_speed; then
        switch_ros2_to_fast_mirror
    fi
}

# ── bashrc 配置工具 ────────────────────────────────────────────────────────────
# 确保指定行存在于 ~/.bashrc，不存在则追加
# 用法：ensure_line_in_bashrc "source /opt/ros/humble/setup.bash"
# 返回：0=已存在或已添加，1=添加失败
ensure_line_in_bashrc() {
    local line="$1"
    local bashrc="$HOME/.bashrc"

    if [[ -z "$line" ]]; then
        log_err "ensure_line_in_bashrc: 参数不能为空"
        return 1
    fi

    # 检查 ~/.bashrc 中是否已存在该行（精确匹配）
    if grep -qxF "$line" "$bashrc" 2>/dev/null; then
        return 0
    fi

    # 追加到 ~/.bashrc
    echo "" >> "$bashrc"
    echo "$line" >> "$bashrc"
    log_ok "已添加到 ~/.bashrc：$line"
    return 0
}
