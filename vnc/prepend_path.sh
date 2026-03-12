#!/bin/bash
# prepend_path.sh - 将目录添加到 PATH 头部（确保在最前面）
# 用法：source prepend_path.sh /path/to/dir

prepend_path() {
    local _dir="$1"
    # 转为绝对路径
    _dir="$(cd "$_dir" 2>/dev/null && pwd)" || return 1

    # 如果已在最前面，无需修改
    case "$PATH" in
        "$_dir":*) ;;
        *)
            # 从 PATH 中移除该目录（无论位置），然后添加到最前面
            PATH=$(printf '%s' "$PATH" | tr ':' '\n' | grep -vxF "$_dir" | tr '\n' ':')
            # 移除末尾多余的冒号
            PATH="${PATH%:}"
            PATH="$_dir:$PATH"
            export PATH
            ;;
    esac

    # 返回解析后的路径（供调用者使用）
    PREPEND_PATH_RESULT="$_dir"
}

# 如果带参数调用，直接执行并输出提示
if [ -n "$1" ]; then
    prepend_path "$1"
    if [ -n "$PREPEND_PATH_RESULT" ]; then
        # echo "✓ 已将目录添加到 PATH 头部: $PREPEND_PATH_RESULT"
    fi
fi