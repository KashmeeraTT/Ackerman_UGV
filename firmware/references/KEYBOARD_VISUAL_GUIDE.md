# Keyboard Teleop Visual Guide

## Keyboard Layout (Standard QWERTY)

```
┌────────────────────────────────────────────────────────────────┐
│ KEYBOARD TELEOP LAYOUT - Ackerman UGV Control                  │
└────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Steering Angle Control (Direct Position)        [NEW]       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│    h         s         y                                    │
│   (-10°)    (0°)     (+10°)                                 │
│                                                              │
│    a                 d                                      │
│   (-2°)     [angle]  (+2°)                                  │
│                                                              │
│  Full Left  Center   Full Right                             │
│   Rotate   Steering   Rotate                                │
│                                                              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Movement Control (Velocity-Based)              [ORIGINAL]   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│      u        i        o                                    │
│    (↖)      (↑)      (↗)                                    │
│                                                              │
│      j        k        l                                    │
│    (←)      (⊘)      (→)                                    │
│                                                              │
│      m        ,        .                                    │
│    (↙)      (↓)      (↘)                                    │
│                                                              │
│  Forward  Forward   Backward                                │
│  Left     Straight  Right                                   │
│                                                              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Speed & Adjustment Controls                    [ORIGINAL]   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  q/z     →   Increase/Decrease ALL speeds (×1.1/×0.9)     │
│  w/x     →   Increase/Decrease LINEAR speed only           │
│  e/c     →   Increase/Decrease ANGULAR speed only          │
│                                                              │
│  k/space →   STOP (zero all velocities)                    │
│  Ctrl+C  →   EXIT program                                  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Control Modes

### Mode 1: Steering Angle Control (Direct)
```
┌──────────────────────────────────┐
│ User presses:  a, d, s, h, y     │
│ Updates:       steering_angle    │
│ Sends:         angular velocity  │
│ ESP32 action:  Move to angle     │
│ Feedback:      Angle display     │
└──────────────────────────────────┘

Example:
  Press 'd' → "Steering: 2.0° | Turn rate: 0.20"
  Press 'd' → "Steering: 4.0° | Turn rate: 0.40"
  Press 's' → "Steering: 0.0° | Turn rate: 0.00"
```

### Mode 2: Movement Control (Velocity)
```
┌──────────────────────────────────┐
│ User presses:  i, j, k, l, etc   │
│ Updates:       x, th (ratio)     │
│ Sends:         linear & angular  │
│ ESP32 action:  Move in direction │
│ Feedback:      None (movement)   │
└──────────────────────────────────┘

Example:
  Press 'i'   → Move forward straight
  Press 'o'   → Move forward + turn right
  Press 'j'   → Spin left in place
  Press 'k'   → Stop completely
```

### Mode 3: Speed Adjustment (Scaling)
```
┌──────────────────────────────────┐
│ User presses:  q, z, w, x, e, c  │
│ Updates:       speed, turn scale │
│ Multiplier:    ×1.1 or ×0.9      │
│ Effect:        Faster/slower     │
│ Feedback:      "Linear: X.XX..."  │
└──────────────────────────────────┘

Example:
  Default: Linear=0.50, Angular=1.00
  Press 'q' → Linear=0.55, Angular=1.10
  Press 'q' → Linear=0.61, Angular=1.21
  Press 'z' → Linear=0.54, Angular=1.09
```

---

## Command Flow Diagram

```
KEYBOARD INPUT
    │
    ├─────────────────────┬──────────────────┬─────────────────┐
    │                     │                  │                 │
    ▼                     ▼                  ▼                 ▼
Steering Keys:        Movement Keys:    Speed Keys:       Special Keys:
(a, d, s, h, y)      (i,j,k,l,u,o,m,.) (q,z,w,x,e,c)    (space, ctrl-c)
    │                     │                  │                 │
    ▼                     ▼                  ▼                 ▼
Update angle         Set x, th ratio    Scale speeds      Stop/Exit
angle_delta = key    x, th = binding    *= multiplier     x=0, th=0
    │                     │                  │                 │
    └──────────┬──────────┴──────────┬───────┴────────────────┘
               │                     │
               ▼                     ▼
         Calculate turn_rate    Use current speeds
         turn_rate = angle/10   Apply to movement
               │                     │
               └──────────┬──────────┘
                          │
                          ▼
                  Create Twist Message
                  linear.x = x * speed
                  angular.z = th * turn
                          │
                          ▼
                  Publish /cmd_vel
                          │
                          ▼
                  ROS2 micro-ROS
                  (Serial transmission)
                          │
                          ▼
                  ESP32 receives Twist
                          │
                ┌─────────┴──────────┐
                │                    │
                ▼                    ▼
           Driving Control    Steering Control
           setVelocity()      setTargetAngle()
                │                    │
                ▼                    ▼
           Smooth ramp         PID controller
           to target velocity  reads encoder
                │                    │
                ▼                    ▼
           Drive motor          Steering motor
           (Smooth accel)       (Precise angle)
```

---

## Key Combination Examples

### Example 1: Forward Right Turn
```
1. Press 'd'  → Steering: 2.0°
2. Press 'd'  → Steering: 4.0°
3. Press 'i'  → Move forward with 4° right turn

Terminal Output:
  Steering: 4.0° | Turn rate: 0.40
  (Then moving forward)
  Linear: 0.50 m/s | Angular: 1.00 rad/s
```

### Example 2: Increase Speed and Turn
```
1. Press 'q'  → Linear: 0.55 m/s | Angular: 1.10 rad/s
2. Press 'w'  → Linear: 0.61 m/s | Angular: 1.10 rad/s
3. Press 'e'  → Linear: 0.61 m/s | Angular: 1.21 rad/s
4. Press 'd'  → Steering: 2.0°
5. Press 'o'  → Forward right (velocity-based)

Result: Moving forward-right at higher speed
```

### Example 3: Spin in Place
```
1. Press 'h'  → Steering: -10.0° (full left)
2. Press ','  → Backward (spinning left)
3. Press 'q'  → Increase spin speed
4. Press 's'  → Center steering
5. Press 'k'  → Stop

Result: UGV spins left, then stops centered
```

### Example 4: Precise Navigation
```
1. Press 's'  → Steering: 0.0° (center)
2. Press 'i'  → Move forward straight
3. Wait 2 sec
4. Press 'd'  → Steering: 2.0°
5. Press 'd'  → Steering: 4.0°
6. Continue forward with 4° right turn
7. Press 's'  → Steering: 0.0° (recenter)
8. Continue forward straight

Result: Drive forward, then turn right, straighten up
```

---

## Real-Time Feedback Display

### Terminal Output Example
```
========================================
UGV Teleop Keyboard Controller
========================================

Movement Controls:
   u    i    o
   j    k    l
   m    ,    .

[... menu continues ...]

Linear: 0.50 m/s | Angular: 1.00 rad/s
Steering: 0.0°

[User presses 'd']
Steering: 2.0° | Turn rate: 0.20

[User presses 'd']
Steering: 4.0° | Turn rate: 0.40

[User presses 'i']
[Teleop sends movement command]

[User presses 'q']
Linear: 0.55 m/s | Angular: 1.10 rad/s

[User presses 'space']
EMERGENCY STOP - All velocities zeroed
Steering: 4.0°
```

### OLED Display Example
```
┌──────────────────────┐
│ UGV STATUS           │
├──────────────────────┤
│ Cal: CALIBRATED      │
│ Ang: 4.0/4.0         │
│ Enc: 200             │
│ Vel: 0.50            │
│ L: OK  R: OK         │
│ ROS2: CONNECTED      │
└──────────────────────┘

(Updates every 200ms)
```

---

## State Machine Diagram

```
                    START
                      │
                      ▼
            ┌─────────────────────┐
            │  Waiting for input   │
            │  steering_angle: 0.0 │
            │  x: 0.0              │
            │  th: 0.0             │
            └──────────┬──────────┘
                       │
         ┌─────────────┼─────────────┐
         │             │             │
         ▼             ▼             ▼
    ┌────────────┐ ┌────────┐ ┌──────────┐
    │Steering    │ │Movement│ │Speed/Stop│
    │Keys        │ │Keys    │ │Keys      │
    │a,d,s,h,y   │ │i,j,... │ │q,z,k,.. │
    └─────┬──────┘ └───┬────┘ └────┬─────┘
          │            │           │
          ▼            ▼           ▼
    Update angle  Set ratio   Scale/Stop
          │            │           │
          └────────┬───┴─────┬─────┘
                   │         │
                   ▼         ▼
           Create & Publish Twist Message
                   │
                   ▼
           Return to Waiting State
```

---

## Encoder Feedback Integration

```
Steering Motor Rotates
         │
         ▼
  Encoder Increments
    (or Decrements)
         │
         ▼
  ISR updates count
  encoderCount++
         │
         ▼
  Calculate angle
  angle = count / 50
         │
         ▼
  PID compares:
  error = target - current
         │
         ▼
  Adjust PWM:
  pwm = PID_output
         │
         ▼
  Motor accelerates/decelerates
  toward target angle
         │
         ▼
  When error ≈ 0:
  Motor stops (maintains angle)
         │
         ▼
  Display shows:
  "Ang: 4.0/4.0" (target reached)
```

---

## Emergency Stop Sequence

```
User presses SPACE or 'k'
         │
         ▼
  teleop sets:
  x = 0.0
  th = 0.0
         │
         ▼
  Publishes:
  Twist(linear.x=0, angular.z=0)
         │
         ▼
  micro-ROS sends to ESP32
         │
         ▼
  ESP32 receives in callback:
  setVelocity(0.0)
  setTargetAngle(0.0)
         │
         ▼
  Controllers update:
  Both ramp to zero
         │
         ▼
  Motors stop (all PWM to 0)
         │
         ▼
  Display shows:
  "EMERGENCY STOP - All velocities zeroed"
  "Steering: X.X°"
```

---

## Typical Usage Pattern

```
1. STARTUP
   ├─ Power on ESP32
   ├─ Wait for calibration
   ├─ See "System Ready!"
   └─ (Optional: Enter steering test with 's')

2. START TELEOP
   └─ ros2 run ugv_teleop teleop_keyboard

3. OPERATE
   ├─ Send steering commands (a, d, s, h, y)
   ├─ Monitor angle display
   ├─ Send movement commands (i, j, k, l, etc)
   ├─ Adjust speed as needed (q, z, w, x, e, c)
   └─ Emergency stop available (space)

4. SHUTDOWN
   ├─ Press 'k' to stop movement
   ├─ Press Ctrl+C to exit teleop
   ├─ Robot stays in safe state
   └─ Power off when done
```

---

## Quick Decision Tree

```
                    What do I want to do?
                            │
        ┌───────────────┬────┴────┬────────────────┐
        │               │         │                │
        ▼               ▼         ▼                ▼
    Move Robot    Change Angle   Adjust Speed   Emergency
    Directions    Precisely      of Movement    Stop
        │               │         │                │
        ▼               ▼         ▼                ▼
    Use i,j,k,l    Use a,d,s,h,y   Use q,z    Press
    u,o,m,.            or w,x,e,c   k/space
        │               │         │                │
        │               │         │                │
    Example:       Example:      Example:       Instantly
    Press 'i'     Press 'd'      Press 'q'      zeroes all
    to move       to increase    to go faster    velocities
    forward       steering
```

---

## Performance Tips

### For Smooth Control
1. **Keyboard input**: Hold key for continuous command
2. **Steering angles**: Use small increments (a/d give ±2°)
3. **Speed adjustment**: Increase gradually (q/z give ±10%)
4. **Movement timing**: Wait for motor to respond (~100ms)

### For Precise Positioning
1. Use direct steering angle commands (a, d, s, h, y)
2. Check encoder feedback (terminal or OLED)
3. Make small adjustments to reach target
4. Verify angle matches target before moving

### For Safe Operation
1. Always center steering (press 's') before high-speed forward
2. Reduce speed before sharp turns
3. Use emergency stop (space) if needed
4. Monitor /cmd_vel topic for verification
