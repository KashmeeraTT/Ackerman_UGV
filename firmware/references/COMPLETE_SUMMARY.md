# Complete Steering Control Implementation Summary

## What Was Implemented

### 1. **Steering Motor Control with Encoder Feedback**
   - Encoder calibration: **500 pulses = 10 degrees**
   - PID control loop for precise angle positioning
   - Real-time encoder feedback (interrupt-driven)
   - Safety limits: ±10° with hardware limit switches
   - Status publishing via ROS2 topic

### 2. **Keyboard Teleop Enhancement**
   - **New steering angle control keys**:
     - `a`/`d`: Decrease/increase steering by 2°
     - `s`: Center steering (0°)
     - `h`/`y`: Full left/right (-10°/+10°)
   - **Existing velocity controls preserved**:
     - Movement keys (i, j, k, l, etc.)
     - Speed adjustment (q, z, w, x, e, c)
   - Real-time steering angle display in terminal

### 3. **Enhanced Display and Feedback**
   - OLED shows current/target steering angle
   - Encoder pulse count display
   - Real-time angle error monitoring
   - Command-received screen with angle feedback

## Files Modified

### Firmware (ESP32)
```
include/
  ├─ pin_config.h              ← Updated encoder calibration constants
  ├─ SteeringController.h       ← Added angle feedback methods
  ├─ MotorDriver.h              ← No changes (working as-is)
  └─ DrivingController.h        ← No changes (working as-is)

src/
  └─ main.cpp                   ← Added testSteeringAngle(), updated displays
```

### ROS2 Teleop
```
ros2_ws/src/ugv_teleop/ugv_teleop/
  └─ teleop_keyboard.py         ← Added steering angle control, new key bindings
```

## Key Features

### Steering Control Flow
```
Keyboard Input
    ↓
Teleop Node (angle command)
    ↓
ROS2 /cmd_vel (Twist message)
    ↓
ESP32 (convert angle/velocity to PWM)
    ↓
Steering Motor + Encoder Feedback
    ↓
PID Control (reach target angle)
    ↓
Motor stops when angle reached
```

### Encoder Specification
```
10 degrees = 500 pulses
Therefore: 50 pulses per degree (50 × 10° = 500)

Angle Calculation:
  angle_degrees = encoder_count / 50

Examples:
  0 pulses    = 0°
  50 pulses   = 1°
  250 pulses  = 5°
  500 pulses  = 10°
```

## Testing & Verification

### Automatic Tests
1. **Limit Switch Test** (on startup)
   - Automatically moves steering left/right to verify switches
   - Displays pass/fail status on OLED

2. **Steering Angle Test** (send 's' on startup)
   - Manual test mode for encoder calibration
   - Commands: `a -2`, `d +2`, `s 0`, `h -10`, `y +10`
   - Shows real-time angle vs encoder pulses

### Manual Testing with Keyboard
```bash
1. ros2 run ugv_teleop teleop_keyboard
2. Send 'd' multiple times to increase steering angle
3. Watch OLED/serial output for:
   - Target angle increasing
   - Encoder count increasing (50 per degree)
   - Motor moving toward target
4. Send 's' to center steering
5. Verify motor returns to center position
```

### ROS2 Topic Monitoring
```bash
# Monitor Twist commands
ros2 topic echo /cmd_vel

# Monitor steering feedback
ros2 topic echo /ugv/status
```

## Documentation Files Created

| File | Purpose |
|------|---------|
| `STEERING_IMPLEMENTATION.md` | Detailed steering control implementation |
| `STEERING_QUICK_REFERENCE.md` | Quick reference for steering test mode |
| `KEYBOARD_TELEOP_GUIDE.md` | Complete keyboard teleop guide |
| `KEYBOARD_QUICK_REFERENCE.md` | Quick keyboard commands reference |
| `INTEGRATION_GUIDE.md` | Complete data flow and integration details |

## Quick Start Guide

### 1. Upload Firmware
```bash
cd /home/thasinduwickrama/Documents/PlatformIO/Projects/Ackerman_UGV
platformio run -e esp32doit-devkit-v1 -t upload
```

### 2. Start ROS2 Environment
```bash
cd ~/ackerman_ugv_ws
source install/setup.bash
```

### 3. Test Steering (Optional)
```bash
# On ESP32 startup, send 's' within 3 seconds
# Commands: a, d, s, h, y, q to quit
```

### 4. Run Keyboard Teleop
```bash
ros2 run ugv_teleop teleop_keyboard

# During operation:
# Steering: a, d, s, h, y (angle control)
# Movement: i, j, k, l, u, o, m, , . (velocity)
# Speed: q, z, w, x, e, c (adjustment)
# Stop: k or space
```

## Control Parameters

### Steering Controller (SteeringController.h)
```cpp
MAX_STEERING_ANGLE_DEG = 10.0      // Hardware limit
MIN_STEERING_ANGLE_DEG = -10.0     // Hardware limit
PULSES_PER_DEGREE = 50             // Encoder calibration
PID Gains: P=2.0, I=0.5, D=0.1     // Tunable
```

### Driving Controller (DrivingController.h)
```cpp
maxVelocity_ = 1.0 m/s             // Configurable
maxAcceleration_ = 0.5 m/s²        // Smooth ramp
```

### Teleop Keyboard (teleop_keyboard.py)
```python
speed = 0.5 m/s (default)          # Adjustable with w/x
turn = 1.0 rad/s (default)         # Adjustable with e/c
steering_angle = ±10°              # Limited by hardware
```

## Performance Metrics

| Metric | Value |
|--------|-------|
| Control Loop Rate | 100 Hz (10ms) |
| Status Publishing | 10 Hz (100ms) |
| Display Update | 5 Hz (200ms) |
| Encoder Resolution | 50 pulses/degree |
| Steering Range | ±10 degrees |
| Max Speed | 2.0 m/s |
| Max Angular Velocity | 3.0 rad/s |
| Limit Switch Debounce | 50 ms |
| Startup Grace Period | 2 seconds |

## Known Limitations & Notes

1. **Movement keys override steering angles**
   - When pressing 'i', 'j', 'l', etc., the steering angle is converted to angular velocity from movement ratio
   - Steering angle state persists but doesn't directly control motor during movement

2. **Keyboard input is blocking**
   - Teleop waits for keypress before sending next message
   - Smooth continuous motion requires holding keys

3. **Encoder counting direction**
   - Quadrature decoding used (A and B channels)
   - Verify rotation direction matches expected encoder count direction

4. **PID Tuning**
   - Default gains (P=2.0, I=0.5, D=0.1) may need adjustment based on motor response
   - Modify in SteeringController.h and recompile

## Troubleshooting

### Common Issues

**Problem**: Steering not moving
- Check encoder pins (34, 35) connections
- Verify motor has power (pins 25, 26)
- Test limit switches with 't' on startup
- Check if emergency stop is active

**Problem**: Encoder count not changing
- Verify encoder mechanical coupling to steering shaft
- Check encoder wiring polarity
- Monitor /ugv/status for encoder field

**Problem**: Steering overshoots target angle
- Reduce PID P gain
- Increase D gain for damping
- Verify encoder resolution (should be 50 pulses/degree)

**Problem**: ROS2 commands not received
- Check micro-ROS initialization: "micro-ROS initialized successfully!"
- Verify /cmd_vel topic: `ros2 topic list`
- Check USB connection and serial port

**Problem**: Keyboard not responding
- Ensure terminal window is in focus
- Check ROS2 environment sourced
- Try quitting (Ctrl+C) and restarting teleop

## Next Steps (Optional Enhancements)

1. **Kalman Filter**: Better angle estimation from noisy encoder
2. **Driving Encoder Integration**: Add closed-loop control for wheels
3. **ROS2 Action Server**: For precise angle positioning from commands
4. **Web Interface**: Browser-based remote control
5. **Autonomous Navigation**: Integrate with Nav2 stack
6. **Logging & Playback**: Record and replay command sequences

## Support & Questions

For issues or questions:
1. Check relevant `.md` file in project root
2. Review ESP32 serial output for error messages
3. Verify hardware connections match pin_config.h
4. Test with manual commands via `ros2 topic pub`

---

## Summary

You now have a complete **Ackerman steering UGV** with:
- ✅ Encoder-based steering angle control
- ✅ Keyboard teleop with direct angle commands
- ✅ Real-time feedback on OLED and ROS2
- ✅ Safe operation with limit switches
- ✅ PID-based motor control
- ✅ Smooth velocity ramping for driving

The system integrates keyboard input → ROS2 messages → ESP32 control → motor feedback in a complete closed-loop system.
