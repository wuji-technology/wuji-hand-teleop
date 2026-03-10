#!/bin/bash
# ============================================================================
# PICO 测试环境清理脚本
# ============================================================================
# 用途：清理所有 PICO 相关的后台进程和 ROS2 节点
# 使用：./cleanup.sh 或 bash cleanup.sh
# ============================================================================

echo "=========================================="
echo "清理 PICO 测试环境"
echo "=========================================="

# 1. 查找并显示所有相关进程
echo ""
echo "查找相关进程..."
echo "------------------------------------------"
ps aux | grep -E "(step4|step5|pico_input|recorded_data|test_pose)" | grep -v grep | grep -v cleanup

# 2. 终止 Python 测试脚本
echo ""
echo "终止测试脚本..."
pkill -9 -f "step4_visualize"
pkill -9 -f "step5_incremental"
pkill -9 -f "test_pose_publisher"
pkill -9 -f "pico_input_node"
sleep 0.5

# 3. 终止 ROS2 静态 TF 发布器
echo "终止静态 TF 发布器..."
pkill -9 -f "static_tf"
sleep 0.5

# 4. 可选：终止 RViz（根据需要取消注释）
# echo "终止 RViz..."
# pkill -9 rviz2

# 5. 验证清理结果
echo ""
echo "=========================================="
echo "清理完成！剩余进程检查："
echo "=========================================="
remaining=$(ps aux | grep -E "(step4|step5|pico_input|recorded_data|test_pose)" | grep -v grep | grep -v cleanup)

if [ -z "$remaining" ]; then
    echo "✓ 所有 PICO 相关进程已清理"
else
    echo "⚠️ 仍有以下进程运行："
    echo "$remaining"
fi

# 6. 检查 ROS2 节点
echo ""
echo "ROS2 活动节点："
echo "------------------------------------------"
timeout 2 ros2 node list 2>&1 || echo "无活动节点"

echo ""
echo "=========================================="
echo "如需重新测试，请运行："
echo "  python3 step4_visualize_recorded_data.py --file ../record/trackingData_sample_static.txt"
echo "=========================================="
