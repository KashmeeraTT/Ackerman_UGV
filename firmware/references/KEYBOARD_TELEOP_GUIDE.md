# Keyboard Teleop Guide - Steering Angle Control Integration

## Overview
The keyboard teleop node now supports both **velocity-based control** (original) and **direct steering angle control** (new), mapped to the UGV's ±10° steering range.

## Keyboard Commands

### Movement Controls (Velocity-Based)
```
   u    i    o
   j    k    l
   m    ,    .
```

| Key | Action | Effect |
|-----|--------|--------|
| `i` | Forward | Linear: +1.0, Angular: 0.0 |
| `o` | Forward-Right | Linear: +1.0, Angular: -1.0 |
| `j` | Turn Left | Linear: 0.0, Angular: +1.0 |
| `l` | Turn Right | Linear: 0.0, Angular: -1.0 |
| `u` | Forward-Left | Linear: +1.0, Angular: +1.0 |
| `,` | Backward | Linear: -1.0, Angular: 0.0 |
| `.` | Backward-Right | Linear: -1.0, Angular: -1.0 |
| `m` | Backward-Left | Linear: -1.0, Angular: +1.0 |
| `k` or `space` | Stop | All velocities: 0.0 |

### Direct Steering Angle Control (NEW)
These commands directly set steering angle in degrees (-10° to +10°):

| Key | Command | Steering Angle |
|-----|---------|-----------------|
| `a` | Decrease angle | Current - 2° |
| `d` | Increase angle | Current + 2° |
| `s` | Center steering | 0° (straight) |
| `h` | Full left | -10° |
| `y` | Full right | +10° |

**Example sequence**:
```
Press 'a' → Steering: -2.0° | Turn rate: -0.20
Press 'a' → Steering: -4.0° | Turn rate: -0.40
Press 'a' → Steering: -6.0° | Turn rate: -0.60
Press 'h' → Steering: -10.0° | Turn rate: -1.00
Press 's' → Steering: 0.0° | Turn rate: 0.00
Press 'd' → Steering: 2.0° | Turn rate: 0.20
Press 'y' → Steering: 10.0° | Turn rate: 1.00
```

### Speed Adjustment
| Key | Action | Effect |
|-----|--------|--------|
| `q` | Increase all speeds | ×1.1 |
| `z` | Decrease all speeds | ×0.9 |
| `w` | Increase linear speed | Linear ×1.1 |
| `x` | Decrease linear speed | Linear ×0.9 |
| `e` | Increase angular speed | Angular ×1.1 |
| `c` | Decrease angular speed | Angular ×0.9 |

**Speed limits**:
- Linear velocity: 0.1 - 2.0 m/s
- Angular velocity: 0.1 - 3.0 rad/s

### Exiting
- `Ctrl+C` - Exit teleop node (gracefully stops UGV)

## Steering Angle to Angular Velocity Mapping

The steering angle is automatically converted to angular velocity:

```
turn_rate = (steering_angle / max_steering_angle) * speed
```

Examples:
- Steering 0° → turn_rate = 0.0
- Steering 5° → turn_rate = 0.5 (turn at 50% of max)
- Steering 10° → turn_rate = 1.0 (turn at 100% of max)
- Steering -10° → turn_rate = -1.0 (turn at -100%)

## Usage Scenarios

### Scenario 1: Simple Forward Movement
```
1. Press 'i'           → Move forward
2. Press 'q'           → Increase speed
3. Press 'l'           → Turn right slightly
4. Press 'k'           → Stop
```

### Scenario 2: Precise Angle Control
```
1. Press 's'           → Center steering (0°)
2. Press 'd'           → Increase to 2°
3. Press 'd'           → Increase to 4°
4. Press 'i'           → Forward with 4° right turn
5. Press 's'           → Recenter steering
6. Press 'k'           → Stop
```

### Scenario 3: Maximum Angle Control
```
1. Press 'y'           → Full right (10°)
2. Press 'i'           → Forward with maximum right turn
3. Press 'h'           → Full left (-10°)
4. Press ',,'           → Backward with maximum left turn
5. Press 's'           → Center steering
```

### Scenario 4: Speed and Angle Combination
```
1. Press 'w'           → Increase linear speed
2. Press 'e'           → Increase angular speed
3. Press 'd' 'd'       → Set steering to 4°
4. Press 'i'           → Forward with 4° right turn at higher speeds
```

## Output Display

The teleop node prints real-time feedback:

```
========================================
UGV Teleop Keyboard Controller
========================================

Movement Controls:
   u    i    o
   j    k    l
   m    ,    .

... (menu) ...

Linear: 0.50 m/s | Angular: 1.00 rad/s
Steering: 0.0°

[User presses 'd']
Steering: 2.0° | Turn rate: 0.20

[User presses 'y']
Steering: 10.0° | Turn rate: 1.00

[User presses 'SPACE']
EMERGENCY STOP - All velocities zeroed
Steering: 10.0°
```

## How It Sends Commands to ESP32

Each keypress sends a ROS2 Twist message to `/cmd_vel`:

```
geometry_msgs/msg/Twist:
  linear:
    x: <calculated linear velocity>
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: <calculated angular velocity based on steering angle>
```

### Example: Direct steering 'd' (increase by 2°)
1. Steering angle increases by 2°
2. Turn rate calculated: `turn_rate = steering_angle / 10.0`
3. Twist message sent with `angular.z = turn_rate * self.turn`
4. ESP32 receives and converts to actual steering angle command

### Example: Movement 'j' (turn left)
1. `self.th = 1.0` (turn left ratio)
2. Twist message sent with `angular.z = 1.0 * self.turn`
3. ESP32 receives angular velocity and converts to steering angle

## Key Integration with ESP32

The teleop keyboard sends **angular velocity** (`twist.angular.z`) to the ESP32, which:

1. **Converts angular velocity to steering angle**:
   ```cpp
   if (abs(linearVel) > 0.01) {
       steeringAngle = atan(angularVel * WHEELBASE / linearVel) * (180.0 / PI);
   }
   ```

2. **Clamps to ±10° limit** (enforced by hardware)

3. **Uses PID control** to move motor to target angle

4. **Reads encoder feedback** (500 pulses = 10°) to verify actual angle

5. **Publishes status** including actual steering angle back to ROS2

## Troubleshooting

### Keys Not Responding
- Check terminal is in focus
- Verify ROS2 network connectivity
- Check `/cmd_vel` topic is being published: `ros2 topic echo /cmd_vel`

### Steering Not Moving
- Verify ESP32 is running and calibrated
- Check `/cmd_vel` messages being received on ESP32 (check serial output)
- Ensure steering motor has power
- Verify limit switches are not triggered

### Steering Overshoots Target
- Check PID tuning on ESP32 (SteeringController.h)
- Verify encoder is counting properly
- Monitor `STEERING_IMPLEMENTATION.md` for encoder feedback issues

### Response Lag
- Increase ROS2 publication rate (default 10 Hz)
- Reduce other background processes
- Check network latency: `ros2 ping /cmd_vel`

## Technical Details

### Files Modified
- `ros2_ws/src/ugv_teleop/ugv_teleop/teleop_keyboard.py` - Added steering angle control

### New Variables in TeleopKeyboard
- `steering_angle` - Current steering angle (-10 to +10°)
- `max_steering_angle` - Maximum allowed angle (10°)
- `min_steering_angle` - Minimum allowed angle (-10°)

### New Methods in TeleopKeyboard
- `update_steering_angle(delta)` - Update steering angle and convert to turn rate

### New Key Bindings
- `steeringBindings` dict maps 'a', 'd', 's', 'h', 'y' to steering commands

## Advanced Usage

### Remote Control Over Network
If ROS2 is running over network (DDS):
```bash
# On remote computer
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.4}}'
```

### Recording Demonstration
```bash
# Record all commands
ros2 bag record /cmd_vel

# Playback later
ros2 bag play rosbag2_2025_12_02
```

### RViz Monitoring
```bash
# View real-time steering angle in RViz
# Subscribe to /ugv/status topic for encoder feedback
ros2 topic echo /ugv/status
```

## Keyboard Layout Reference

```
┌─────────────────────────────────────┐
│  Directional (Movement)             │
│  u(↖) i(↑) o(↗)                    │
│  j(←) k(0) l(→)                    │
│  m(↙) ,(↓) .(↘)                    │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  Steering Angle (Direct Control)    │
│  h(←10°)  s(0°)   y(→10°)         │
│  a(-2°) [current]  d(+2°)         │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  Speed Adjustment                   │
│  q/z (both), w/x (linear)          │
│  e/c (angular)                      │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  Special                            │
│  Space/k (Stop), Ctrl+C (Exit)     │
└─────────────────────────────────────┘
```
