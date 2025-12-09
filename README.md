# Robot Workspace Documentation

This workspace contains the complete software stack for a mobile robot using **Orbbec Gemini 2L** for Visual SLAM and **Nav2** for autonomous navigation.


## 1. Environment Setup

### Prerequisites
- **Ubuntu 22.04 LTS**
- **ROS 2 Humble Hawksbill** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))

### Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/KashmeeraTT/Ackerman_UGV.git robot_ws
   cd robot_ws
   ```

2. **Install Dependencies**
   We use `rosdep` to install system dependencies.
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Install Build Tools (if not present)**
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

## 2. Build Instructions

Build the entire workspace using `colcon`.

```bash
colcon build --symlink-install
```

*Note: If you encounter symlink errors (common with some filesystems), try `colcon build` without the `--symlink-install` flag, or clean the build folder with `rm -rf build install log` and retry.*

## 3. System Architecture

The system uses ORB-SLAM3 for localization (generating Odometry) and Nav2 for path planning and control.

```mermaid
graph TD
    Camera[Orbbec Gemini 2L] -->|RGB+Depth Images| SPL[perception_vslam]
    SPL -->|/orbslam3/pose| Bridge[slam_odom_bridge]
    Bridge -->|/odom + TF| Nav2[Nav2 Stack]
    Nav2 -->|/cmd_vel| Robot[Robot Base]
```

## 2. Package Overview

| Package Name | Purpose | Key Launch/Nodes |
| :--- | :--- | :--- |
| **robot_bringup** | Top-level entry point for the whole system. | `system.launch.py` |
| **perception_vslam** | Handles VSLAM startup and integration. | `vslam_bringup.launch.py` |
| **orbslam3_ros2** | Core Visual SLAM wrapper for ORB-SLAM3. | `orbslam3_ros2` node |
| **slam_odom_bridge** | Converts SLAM Poses to standard ROS 2 Odometry. | `slam_odom_bridge` node |
| **nav2_bringup_ack** | Navigation 2 stack configuration and launch. | `nav2_bringup.launch.py` |
| **OrbbecSDK_ROS2** | Drivers for Orbbec cameras. | `orbbec_camera` |
| **ackermann_bridge_demo**| Demonstrations/Bridge for vehicle control. | - |
| **ugv_teleop** | Teleoperation tools. | - |

## 3. Dependencies

- **ROS 2 Humble Hawksbill**
- **Navigation 2 (`nav2_bringup`, `navigation2`)**
- **OrbbecSDK**
- **Pangolin** (for ORB-SLAM3)
- **OpenCV**

## 5. How to Run

### Source the Workspace
Before running any command, always source the install setup:
```bash
source install/setup.bash
```

### Quick Start (All-in-One)
Launch the entire system including VSLAM, Navigation, and RViz:
```bash
ros2 launch robot_bringup system.launch.py
```
*Note: To disable auto-launch of RViz, use: `ros2 launch robot_bringup system.launch.py use_rviz:=false`*

### Manual Start (Component-wise)
For debugging or running specific parts:

**1. VSLAM & Camera only**
```bash
ros2 launch perception_vslam vslam_bringup.launch.py
```

**2. Navigation only**
```bash
ros2 launch nav2_bringup_ack nav2_bringup.launch.py
```

**3. Visualization**
```bash
ros2 launch robot_bringup rviz.launch.py
# OR manually:
ros2 run rviz2 rviz2 -d src/robot_bringup/rviz/robot.rviz
```

## 6. Troubleshooting
- **No Map**: Ensure SLAM is tracking (check debug window if enabled).
- **TF Error**: Verify `static_transform_publisher` is running for camera link.
- **Robot Stalls**: Check `cmd_vel` output and safety limits in `nav2_params.yaml`.
