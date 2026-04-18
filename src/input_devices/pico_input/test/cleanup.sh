#!/bin/bash
# ============================================================================
# PICO Test Environment Cleanup Script
# ============================================================================
# Purpose: Clean up all PICO-related background processes and ROS2 nodes
# Usage: ./cleanup.sh or bash cleanup.sh
# ============================================================================

echo "=========================================="
echo "Cleaning up PICO test environment"
echo "=========================================="

# 1. Find and display all related processes
echo ""
echo "Finding related processes..."
echo "------------------------------------------"
ps aux | grep -E "(step4|step5|pico_input|recorded_data|test_pose)" | grep -v grep | grep -v cleanup

# 2. Terminate Python test scripts
echo ""
echo "Terminating test scripts..."
pkill -9 -f "step4_visualize"
pkill -9 -f "step5_incremental"
pkill -9 -f "test_pose_publisher"
pkill -9 -f "pico_input_node"
sleep 0.5

# 3. Terminate ROS2 static TF broadcasters
echo "Terminating static TF broadcasters..."
pkill -9 -f "static_tf"
sleep 0.5

# 4. Optional: Terminate RViz (uncomment if needed)
# echo "Terminating RViz..."
# pkill -9 rviz2

# 5. Verify cleanup results
echo ""
echo "=========================================="
echo "Cleanup complete! Checking remaining processes:"
echo "=========================================="
remaining=$(ps aux | grep -E "(step4|step5|pico_input|recorded_data|test_pose)" | grep -v grep | grep -v cleanup)

if [ -z "$remaining" ]; then
    echo "All PICO-related processes have been cleaned up"
else
    echo "The following processes are still running:"
    echo "$remaining"
fi

# 6. Check ROS2 nodes
echo ""
echo "ROS2 active nodes:"
echo "------------------------------------------"
timeout 2 ros2 node list 2>&1 || echo "No active nodes"

echo ""
echo "=========================================="
echo "To re-run tests, execute:"
echo "  python3 step4_visualize_recorded_data.py --file ../record/trackingData_sample_static.txt"
echo "=========================================="
