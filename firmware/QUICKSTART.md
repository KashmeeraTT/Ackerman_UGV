# UGV Quick Start Guide

## First Time Setup

### 1. Build ESP32 Firmware
```bash
cd /home/thasinduwickrama/Documents/PlatformIO/Projects/Ackerman_UGV
./build_esp32.sh
```
- Press `y` to upload
- Press `y` to monitor calibration

### 2. Build ROS2 Workspace
```bash
./build_ros2.sh
```

### 3. Install micro-ROS Agent (if not installed)
```bash
sudo apt update
sudo apt install ros-humble-micro-ros-agent
```

---

## Daily Operation

### Launch System
```bash
./launch_ugv.sh
```

This will:
1. Auto-detect ESP32 serial port
2. Start micro-ROS agent
3. Start teleop keyboard
4. Show control instructions

---

## Keyboard Controls

```
   u    i    o      Movement:
   j    k    l      i = forward, k = stop, , = backward
   m    ,    .      j/l = turn left/right
```

**Speed Controls:**
- `q/z` - Increase/decrease all speeds
- `space` - Emergency stop
- `Ctrl+C` - Quit

---

## Monitoring

### View UGV Status
```bash
# In a new terminal
cd ros2_ws
source install/setup.bash
ros2 topic echo /ugv/status
```

### View cmd_vel Commands
```bash
ros2 topic echo /cmd_vel
```

### View All Topics
```bash
ros2 topic list
```

---

## Troubleshooting

### ESP32 Not Connecting
1. Check USB cable
2. Verify port: `ls /dev/ttyUSB*`
3. Reset ESP32 (press button on board)
4. Check micro-ROS agent is running

### Emergency Stop Active
1. Check limit switches (may be triggered)
2. Power cycle the UGV
3. Let recalibration run

### Steering Not Responding
```bash
# Monitor serial output
pio device monitor
```
Look for:
- "Calibration successful!"
- "System ready!"
- E-Stop status

---

## File Locations

- **ESP32 Firmware**: `src/main.cpp`
- **Pin Config**: `include/pin_config.h`
- **ROS2 Teleop**: `ros2_ws/src/ugv_teleop/`
- **Documentation**: `README.md`

---

## Safety Notes

⚠️ **Max Steering**: ±10° (hardware limit)
⚠️ **Emergency Stop**: Space bar or limit switches
⚠️ **Test Elevated**: Always test with wheels off ground first

---

## Support

- Check `README.md` for detailed documentation
- Check `walkthrough.md` for implementation details
- Monitor `/ugv/status` topic for debugging
