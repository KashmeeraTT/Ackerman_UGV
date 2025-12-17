#!/bin/bash
# Launch UGV Teleop System

set -e

echo "========================================"
echo "UGV Teleop Control System"
echo "========================================"

cd "$(dirname "$0")/ros2_ws"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "ERROR: ROS2 workspace not built!"
    echo "Please run ./build_ros2.sh first"
    exit 1
fi

echo ""
echo "Detecting serial port..."

# Try to find ESP32 serial port
if [ -e /dev/ttyUSB0 ]; then
    SERIAL_PORT="/dev/ttyUSB0"
elif [ -e /dev/ttyACM0 ]; then
    SERIAL_PORT="/dev/ttyACM0"
else
    echo "Could not auto-detect serial port."
    echo "Available ports:"
    ls /dev/tty* 2>/dev/null | grep -E "(USB|ACM)" || echo "  None found"
    echo ""
    read -p "Enter serial port (e.g., /dev/ttyUSB0): " SERIAL_PORT
fi

echo "Using serial port: $SERIAL_PORT"
echo ""
echo "Starting system..."
echo "  - micro-ROS Agent"
echo "  - Teleop Keyboard"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================"
echo ""

# Source workspace
source install/setup.bash

# Launch
ros2 launch ugv_teleop ugv_teleop.launch.py
