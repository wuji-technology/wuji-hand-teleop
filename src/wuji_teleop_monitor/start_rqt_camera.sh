#!/bin/bash
# ============================================================================
# 一键启动 rqt 相机监控
# ============================================================================
#
# 功能说明:
#   在一个 rqt 窗口中自动加载 3 个相机视图
#   - 立体相机左眼
#   - 左腕相机
#   - 右腕相机
#
# 使用方法:
#   ./start_rqt_camera.sh
#
# 启动后:
#   - 会自动显示 3 个相机画面
#   - 可以拖拽调整布局
#   - 按 Ctrl+C 或关闭窗口退出
#
# ============================================================================

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 从脚本位置推断工作空间路径 (向上3层: wuji_teleop_monitor -> src -> wuji-teleop-ros2-private -> ros2_ws)
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Source ROS 2 基础环境
source /opt/ros/humble/setup.bash

# Source 工作空间 (如果存在)
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    echo "已加载工作空间: ${WS_DIR}"
else
    echo "警告: 未找到工作空间 ${WS_DIR}/install/setup.bash"
fi

echo "启动相机监控..."
echo ""
echo "相机话题:"
echo "  - /stereo/left/compressed (立体相机左眼)"
echo "  - /cam_left_wrist/color/image_rect_raw/compressed (左腕)"
echo "  - /cam_right_wrist/color/image_rect_raw/compressed (右腕)"
echo ""
echo "布局: 立体相机在上，左右腕相机在下"
echo "按 'q' 或 ESC 键退出"
echo ""

# 启动 Python 相机查看器
python3 "${SCRIPT_DIR}/camera_viewer.py"

