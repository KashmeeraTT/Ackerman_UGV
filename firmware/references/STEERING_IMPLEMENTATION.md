# Steering Control with Encoder Feedback Implementation

## Overview
Implemented steering motor control with encoder-based angle feedback. The system now actively controls the steering angle and reads encoder pulses to verify the actual position.

## Key Specifications
- **Encoder Calibration**: 500 pulses = 10 degrees of steering rotation
- **Pulses per degree**: 50 pulses/degree
- **Steering range**: -10° to +10° (20° total range)
- **Control method**: PID control loop with encoder feedback

## Changes Made

### 1. **pin_config.h**
Updated encoder configuration constants:
```cpp
#define PULSES_PER_10_DEGREES 500          // 10 degrees = 500 pulses
#define PULSES_PER_DEGREE (PULSES_PER_10_DEGREES / 10.0) // 50 pulses/degree
```

### 2. **SteeringController.h**
Added new methods for angle feedback:
- `getSteeringAngleFromEncoder()`: Returns actual steering angle calculated from encoder pulses
- `getTargetSteeringAngle()`: Returns the target angle being commanded

These complement the existing:
- `getCurrentAngle()`: Returns PID-controlled current angle
- `getTargetAngle()`: Returns the target angle setpoint
- `getEncoderCount()`: Returns raw encoder pulse count
- `setTargetAngle(angleDeg)`: Command steering to a specific angle in degrees

### 3. **main.cpp** - New Steering Test Function
Added `testSteeringAngle()` function for manual testing:

```
Commands in test mode:
  a 5   - Move to +5 degrees
  a -5  - Move to -5 degrees
  a 0   - Move to center (0 degrees)
  s     - Show current angle
  q     - Quit test mode
```

**How to use**:
1. Upload code to ESP32
2. Open Serial Monitor at 115200 baud
3. During startup, send 's' within 3 seconds to enter steering angle test mode
4. Send angle commands to test steering motor and encoder feedback

### 4. **Display Updates**
Enhanced OLED display to show:
- Current steering angle (from PID control)
- Target steering angle
- Encoder pulse count in real-time
- Comparison between target and actual angle

In command-received display:
- Target angle in degrees
- Current angle in degrees

## How Steering Control Works

### During Calibration
1. Motor moves LEFT until limit switch triggers
2. Motor moves RIGHT until limit switch triggers  
3. Center position is calculated as midpoint
4. Encoder count is reset to 0 at center

### During Normal Operation
1. ROS2 command (`/cmd_vel`) provides linear and angular velocity
2. Angular velocity is converted to target steering angle
3. PID controller continuously adjusts motor output to reach target angle
4. Encoder feedback (via interrupt) updates current angle in real-time
5. Motor stops when angle matches target (within PID tolerance)

### Encoder Reading
- Encoder ISR counts pulses on every edge transition (CHANGE)
- Quadrature decoding determines direction:
  - Forward: `encoderCount_++`
  - Reverse: `encoderCount_--`
- Current angle = `encoderCount / PULSES_PER_DEGREE`

### Safety Features
- Hard limits at ±10° (enforced by limit switches)
- Emergency stop triggers if steering exceeds limits
- Debouncing on limit switches (50ms)
- Startup grace period to prevent spurious triggers
- PID output limits to ±255 PWM range

## Testing the Implementation

### 1. Encoder Calibration Test
Send 's' on startup to enter steering angle test mode:
```
Target: 5.0 deg | Current: 0.0 deg | Encoder: 0
Target: 5.0 deg | Current: 1.2 deg | Encoder: 60
Target: 5.0 deg | Current: 4.8 deg | Encoder: 240
Target: 5.0 deg | Current: 5.0 deg | Encoder: 250
```
Watch the encoder count increase (50 counts per degree)

### 2. Full Range Test
```
a -10    # Move fully left
a 0      # Return to center
a 10     # Move fully right
a 0      # Return to center
```

### 3. ROS2 Integration
Once system initializes, send commands via ROS2:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```
Monitor OLED or serial output to see steering angle being controlled

## Troubleshooting

### Encoder not counting
- Check encoder pins 34 and 35 connections
- Verify encoder is mechanically coupled to steering shaft
- Check encoder ISR is firing (add debug prints in `handleEncoderISR`)

### Steering not reaching target angle
- Verify PID tuning (P=2.0, I=0.5, D=0.1)
- Check motor can move freely without mechanical binding
- Monitor encoder pulses to confirm they're being detected
- Verify motor PWM output (pins 25, 26)

### Limit switches triggering unexpectedly
- Check wiring - switches should be GND-connected (active LOW)
- Verify pull-up resistors are enabled (INPUT_PULLUP mode)
- Check debounce timing (LIMIT_DEBOUNCE_MS = 50ms)
- Ensure steering is not physically at limits

## Performance Metrics

| Metric | Value |
|--------|-------|
| Steering range | ±10° |
| Encoder resolution | 50 pulses/° |
| PID update rate | 100 Hz |
| Control period | 10 ms |
| Encoder debounce | 50 ms |
| Limit switch debounce | 50 ms |
| Startup grace period | 2 seconds |

## Next Steps (Optional Enhancements)

1. **PID Tuning**: Adjust P/I/D values based on actual motor response
2. **Driving Encoder Integration**: Add feedback control for wheel encoders
3. **Kalman Filter**: Implement for better angle estimation
4. **ROS2 Feedback**: Publish actual steering angle to ROS2 topic
5. **Velocity Ramp**: Add smooth acceleration/deceleration for steering

## Files Modified
- `/include/pin_config.h` - Encoder calibration constants
- `/include/SteeringController.h` - Added angle feedback methods
- `/src/main.cpp` - Added steering angle test function, enhanced displays
