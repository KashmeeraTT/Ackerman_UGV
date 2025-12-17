#!/bin/bash
# Build ROS2 Workspace

set -e

echo "========================================"
echo "ROS2 Workspace Build"
echo "========================================"

cd "$(dirname "$0")/ros2_ws"

echo ""
echo "Building ROS2 packages..."
colcon build

echo ""
echo "Build complete!"
echo ""
echo "To use the workspace, run:"
echo "  source ros2_ws/install/setup.bash"
echo ""
