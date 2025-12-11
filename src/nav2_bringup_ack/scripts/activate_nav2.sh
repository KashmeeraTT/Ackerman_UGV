#!/bin/bash

# Nav2 Lifecycle Nodes Activation Script
# This script activates all Nav2 lifecycle nodes after system startup
# since the lifecycle_manager's autostart feature is unreliable

echo "[Nav2 Activation] Waiting 8 seconds for all nodes to initialize..."
sleep 8

echo "[Nav2 Activation] Activating Nav2 lifecycle nodes via lifecycle_manager..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/xavier/robot_ws/install/setup.bash

# Call the lifecycle manager service to startup (configure + activate) all nodes
# Command 1 = startup (configure then activate)
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"

if [ $? -eq 0 ]; then
    echo "[Nav2 Activation] SUCCESS: Nav2 nodes activated!"
    
    # Verify activation
    echo "[Nav2 Activation] Verifying bt_navigator state..."
    STATE=$(ros2 lifecycle get /bt_navigator 2>&1 | grep -o "active\|inactive\|unconfigured")
    echo "[Nav2 Activation] bt_navigator state: $STATE"
else
    echo "[Nav2 Activation] FAILED: Could not activate Nav2 nodes"
    exit 1
fi
