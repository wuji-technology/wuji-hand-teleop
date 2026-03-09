#!/bin/bash

# =============================================================================
# Teleop Monitor 桌面快捷方式安装脚本
# =============================================================================
#
# 使用方法:
#   cd ~/ros2_ws/src/wuji-hand-teleop-ros2/src/wuji_teleop_monitor
#   ./install_desktop.sh
#
# 功能:
#   - 在用户桌面创建 Teleop Monitor 快捷方式
#   - 自动检测桌面路径 (支持中文"桌面"和英文"Desktop")
#   - 自动设置正确的路径和权限
#
# =============================================================================

set -e

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 获取用户桌面路径
DESKTOP_DIR=$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")

# 检查桌面目录是否存在
if [ ! -d "$DESKTOP_DIR" ]; then
    echo "错误: 桌面目录不存在: $DESKTOP_DIR"
    exit 1
fi

# 包内 Python 模块路径 (logo 所在位置)
PACKAGE_PATH="$SCRIPT_DIR/wuji_teleop_monitor"

# 检查 logo 是否存在
if [ ! -f "$PACKAGE_PATH/wuji.jpg" ]; then
    echo "警告: Logo 文件不存在: $PACKAGE_PATH/wuji.jpg"
fi

# 检查模板文件
TEMPLATE_FILE="$SCRIPT_DIR/teleop-monitor.desktop.template"
if [ ! -f "$TEMPLATE_FILE" ]; then
    echo "错误: 模板文件不存在: $TEMPLATE_FILE"
    exit 1
fi

# 生成 .desktop 文件
OUTPUT_FILE="$DESKTOP_DIR/teleop-monitor.desktop"

sed -e "s|{{HOME}}|$HOME|g" \
    -e "s|{{PACKAGE_PATH}}|$PACKAGE_PATH|g" \
    "$TEMPLATE_FILE" > "$OUTPUT_FILE"

# 添加执行权限
chmod +x "$OUTPUT_FILE"

# 标记为可信任 (GNOME 桌面需要)
gio set "$OUTPUT_FILE" metadata::trusted true 2>/dev/null || true

echo "桌面快捷方式已安装到: $OUTPUT_FILE"
echo "完成!"
