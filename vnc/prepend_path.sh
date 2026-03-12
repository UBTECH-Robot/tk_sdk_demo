#!/bin/bash
# prepend_path.sh - 将目录添加到 PATH 头部（避免重复）
# 用法：source prepend_path.sh /path/to/dir

prepend_path() {
    local _dir="$1"
    # 转为绝对路径
    _dir="$(cd "$_dir" 2>/dev/null && pwd)" || return 1

    # 避免重复添加
    case ":$PATH:" in
        *":$_dir:"*) ;;
        *) PATH="$_dir:$PATH"; export PATH ;;
    esac

    # 返回解析后的路径（供调用者使用）
    PREPEND_PATH_RESULT="$_dir"
}

# 如果带参数调用，直接执行并输出提示
if [ -n "$1" ]; then
    prepend_path "$1"
    if [ -n "$PREPEND_PATH_RESULT" ]; then
        echo "✓ 已将目录添加到 PATH: $PREPEND_PATH_RESULT"
        echo ""
        echo "若需在其他终端手动添加，请执行："
        echo "  export PATH=\"$PREPEND_PATH_RESULT:\$PATH\""
    fi
fi