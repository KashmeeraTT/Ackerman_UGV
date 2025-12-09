# Steering Control - Quick Reference

## Encoder Calibration
**500 pulses = 10 degrees** of steering rotation

## Test Steering Angle (Manual Testing)

### Entry
1. Power on ESP32
2. Open Serial Monitor (115200 baud)
3. Send 's' within 3 seconds of startup

### Commands
```
a 5       # Move steering to +5 degrees
a -5      # Move steering to -5 degrees  
a 0       # Move steering to center (0 degrees)
a -10     # Move to full left limit
a 10      # Move to full right limit
s         # Show current angle (refresh display)
q         # Quit test mode
```

### Example Output
```
Target: 5.0 deg | Current: 0.0 deg | Encoder: 0
Target: 5.0 deg | Current: 2.5 deg | Encoder: 125
Target: 5.0 deg | Current: 5.0 deg | Encoder: 250
Target: 0.0 deg | Current: 5.0 deg | Encoder: 250
Target: 0.0 deg | Current: 2.5 deg | Encoder: 125
Target: 0.0 deg | Current: 0.0 deg | Encoder: 0
```

## Real-Time Monitoring

### OLED Display Shows
- **Cal**: Calibration status (CALIBRATED if ready)
- **Ang**: Current/Target angle (e.g., 2.5/5.0)
- **Enc**: Encoder pulse count (multiply by 0.02 to get degrees)
- **Vel**: Driving velocity
- **Limit switches**: L: OK/HIT, R: OK/HIT
- **ROS2 status**: CONNECTED/IDLE/WAITING

### Serial Output
During operation, the steering test displays:
```
Target: 3.5 deg | Current: 3.2 deg | Encoder: 160
Target: 3.5 deg | Current: 3.5 deg | Encoder: 175
```

## Steering Angle from Encoder
```
angle_degrees = encoder_count / 50
or
angle_degrees = encoder_count / PULSES_PER_DEGREE
```

Examples:
- 0 counts = 0°
- 50 counts = 1°
- 250 counts = 5°
- 500 counts = 10°

## ROS2 Control (After Initialization)

Send commands via ROS2:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

The controller will:
1. Convert angular velocity to steering angle
2. Command motor to reach that angle
3. Display actual vs target angle on OLED
4. Stop motor when angle is reached

## Troubleshooting

| Issue | Check |
|-------|-------|
| Encoder not counting | Pins 34, 35; Mechanical coupling |
| Steering not moving | Motor power; Limit switches not triggered |
| Limit switch triggered at startup | Physical alignment; Check GND connection |
| Steering overshoots target | PID tuning needed (P=2.0, I=0.5, D=0.1) |
| Slow response | Check PWM output on pins 25, 26 |

## Pin References
- **Steering Motor PWM**: Pins 25 (LPWM), 26 (RPWM)
- **Steering Encoder**: Pins 34 (A), 35 (B)
- **Limit Switches**: Pins 32 (Left), 33 (Right) - GND connected
- **Serial**: RX=3, TX=1 (115200 baud)
