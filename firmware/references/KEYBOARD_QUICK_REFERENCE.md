# Keyboard Teleop Quick Reference Card

## Direct Steering Angle Control (NEW)
```
┌──────────────────────────────────────┐
│ STEERING ANGLE (-10° to +10°)        │
├──────────────────────────────────────┤
│  h = -10°  │  s = 0°  │  y = +10°    │
│  a = -2°   │ current  │  d = +2°     │
└──────────────────────────────────────┘
```

## Movement Controls (Velocity-Based)
```
┌──────────────────────────────────────┐
│ DIRECTIONAL MOVEMENT                 │
├──────────────────────────────────────┤
│  u ↖  │  i ↑  │  o ↗                 │
│  j ←  │  k ⊘  │  l →                 │
│  m ↙  │  , ↓  │  . ↘                 │
└──────────────────────────────────────┘

u/o = forward + left/right
i = forward
j/l = turn left/right
k/space = stop
m/. = backward + left/right
, = backward
```

## Speed Adjustment
```
q = increase all (×1.1)     w = increase linear (×1.1)
z = decrease all (×0.9)     x = decrease linear (×0.9)
e = increase angular (×1.1) c = decrease angular (×0.9)
```

## Example Sequences

### Scenario 1: Turn Right and Move Forward
```
1. Press 'd' (2° right)
2. Press 'd' (4° right)
3. Press 'i' (move forward with 4° right turn)
```

### Scenario 2: Make a Sharp Left Turn
```
1. Press 'h' (full left: -10°)
2. Press ',' (backward with full left turn)
```

### Scenario 3: Navigate with Precision
```
1. Press 'q' q (increase speed)
2. Press 's' (center steering)
3. Press 'i' (forward straight)
4. Press 'd' (turn 2° right)
5. Press 'd' (turn 4° right)
```

## Real-Time Output
```
Linear: 0.50 m/s | Angular: 1.00 rad/s
Steering: 4.0° | Turn rate: 0.40

↑ Updated every keystroke
```

## Starting Teleop
```bash
# From ROS2 workspace
ros2 run ugv_teleop teleop_keyboard

# Or with custom parameters
ros2 run ugv_teleop teleop_keyboard --ros-args \
  -p speed:=0.3 \
  -p turn:=0.8
```

## Exit
```
Press Ctrl+C to stop and exit
```

## Key Mapping Summary

| Category | Keys | Purpose |
|----------|------|---------|
| **Steering** | h, a, s, d, y | Direct angle control (±10°) |
| **Movement** | u, i, o, j, k, l, m, , . | Velocity-based control |
| **Speed** | q, z, w, x, e, c | Adjust speeds |
| **Stop** | k, space | Stop movement |
| **Exit** | Ctrl+C | Exit program |

## Angle Conversion to Turn Rate

```
Steering Angle  →  Turn Rate (rad/s)
    -10°       →      -1.00
     -5°       →      -0.50
      0°       →       0.00
     +5°       →      +0.50
    +10°       →      +1.00
```

The actual angular velocity sent = `turn_rate × angular_speed_setting`

## Default Speed Settings

| Parameter | Default | Min | Max | Adjustment |
|-----------|---------|-----|-----|------------|
| Linear (m/s) | 0.5 | 0.1 | 2.0 | w/x or q/z |
| Angular (rad/s) | 1.0 | 0.1 | 3.0 | e/c or q/z |

## Hardware Integration

```
Keyboard Input
      ↓
ROS2 /cmd_vel Topic
      ↓
ESP32 (micro-ROS)
      ↓
Steering: angle_cmd + PID control + encoder feedback
Driving: velocity_cmd + smooth acceleration
      ↓
Motor outputs (PWM)
```
