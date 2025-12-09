# Complete Integration: Keyboard Teleop → Steering Control

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│ ROS2 Environment (Linux/Windows/Mac)                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────────────────────────────────────┐                │
│  │ Keyboard Teleop Node (teleop_keyboard.py)   │                │
│  │                                              │                │
│  │ Reads: Keyboard input (a, d, s, h, y, etc) │                │
│  │ Outputs: /cmd_vel topic (Twist messages)    │                │
│  │ Features:                                   │                │
│  │ - Direct steering angle control (±10°)     │                │
│  │ - Velocity-based movement                   │                │
│  │ - Speed adjustment                          │                │
│  │ - Real-time angle display                   │                │
│  └──────────────────┬──────────────────────────┘                │
│                     │                                             │
│                     │ publishes /cmd_vel                         │
│                     │ (Twist: linear.x, angular.z)              │
│                     ▼                                             │
│  ┌─────────────────────────────────────────────┐                │
│  │ micro-ROS (DDS/Serial Bridge)               │                │
│  │                                              │                │
│  │ Transmits Twist messages over serial/USB    │                │
│  └──────────────────┬──────────────────────────┘                │
│                     │ USB-Serial                                 │
└─────────────────────┼──────────────────────────────────────────┘
                      │
                      │ (115200 baud)
                      │
         ┌────────────▼────────────┐
         │  ESP32 (DOIT DevKit V1) │
         │  Ackerman UGV           │
         └────────────┬────────────┘
                      │
      ┌───────────────┼───────────────┐
      │               │               │
      ▼               ▼               ▼
┌───────────┐  ┌────────────┐  ┌──────────────┐
│DrivingCtl │  │SteeringCtl │  │MotorDrivers  │
│           │  │            │  │              │
│- Velocity │  │- Angle PID │  │- LPWM/RPWM   │
│- Smooth   │  │- Encoder   │  │- Speed ctrl  │
│  accel    │  │- Limits    │  │              │
└─────┬─────┘  └──────┬─────┘  └────┬─────────┘
      │               │             │
      │               │         ┌───┴────────┐
      │               │         │            │
      ▼               ▼         ▼            ▼
   Driving        Steering   Steering     Driving
   Motor          Motor      Encoder      Encoder
```

## Data Flow: Single Keystroke Example

### Example: User presses 'd' (increase steering by 2°)

```
1. KEYBOARD INPUT
   └─> User presses 'd'

2. TELEOP KEYBOARD (teleop_keyboard.py)
   ├─> Detects key 'd' in steeringBindings
   ├─> Calls update_steering_angle(+2)
   ├─> steering_angle: 0.0° → 2.0°
   ├─> Calculates turn_rate: 2.0 / 10.0 = 0.2
   ├─> Sets self.th = 0.2
   ├─> Prints: "Steering: 2.0° | Turn rate: 0.20"
   └─> Calls publish_twist(x=0.0, th=0.2)

3. ROS2 TWIST MESSAGE CREATION
   └─> Creates geometry_msgs/msg/Twist:
       {
         linear: {x: 0.0 * 0.5 = 0.0 m/s, y: 0.0, z: 0.0},
         angular: {x: 0.0, y: 0.0, z: 0.2 * 1.0 = 0.2 rad/s}
       }

4. MICRO-ROS TRANSMISSION
   └─> Message sent over /cmd_vel topic
   └─> Transmitted via USB-Serial at 115200 baud

5. ESP32 RECEPTION (main.cpp)
   ├─> Receives Twist message in cmdVelCallback()
   ├─> Extracts: linearVel = 0.0 m/s, angularVel = 0.2 rad/s
   └─> Processes steering command

6. STEERING CONVERSION (main.cpp → SteeringController)
   ├─> Converts angular velocity to steering angle
   ├─> Calculates: steeringAngle = atan(0.2 * WHEELBASE / 0.01) * 180/π
   ├─> Clamps to ±10°
   └─> Calls steeringController.setTargetAngle(steeringAngle)

7. STEERING CONTROL (SteeringController.h)
   ├─> Sets targetAngle_ = calculated angle
   ├─> Main loop calls update()
   ├─> PID computes: pidOutput_ based on (targetAngle - currentAngle)
   ├─> Motor receives PWM command: setSpeed(pidOutput)
   └─> Motor starts moving toward target angle

8. ENCODER FEEDBACK (Interrupt-driven)
   ├─> Encoder counts pulses from motor shaft
   ├─> ISR updates encoderCount_ (increments/decrements)
   ├─> updateCurrentAngle() converts: currentAngle = count / 50
   ├─> PID continuously adjusts based on error
   └─> Motor stops when angle reached (within PID tolerance)

9. STATUS PUBLISHING (main.cpp)
   ├─> Every 100ms, publishStatus()
   ├─> Publishes to /ugv/status topic:
   │   "Angle:2.00 Target:2.00 Vel:0.00 Enc:100 E-Stop:0"
   └─> ROS2 can monitor steering angle in real-time

10. DISPLAY UPDATE (main.cpp → OLED)
    ├─> Updates OLED display every 200ms
    ├─> Shows:
    │   "Ang: 2.0/2.0" (current/target)
    │   "Enc: 100"
    └─> User sees live feedback
```

## Multiple Key Sequence: Forward with Steering

### Example: User presses 'd', 'd', 'i'

```
Keypress 1: 'd'
├─ steering_angle: 0.0° → 2.0°
├─ turn_rate: 0.2
└─ Twist: linear.x=0.0, angular.z=0.2

Keypress 2: 'd'
├─ steering_angle: 2.0° → 4.0°
├─ turn_rate: 0.4
└─ Twist: linear.x=0.0, angular.z=0.4

Keypress 3: 'i'
├─ moveBindings['i'] = (1, 0)
├─ Sets: self.x=1.0, self.th=0.0
├─ Resets turn_rate to 0.0 (from movement)
├─ steering_angle still = 4.0°
└─ Twist: linear.x=0.5, angular.z=0.0

Note: Movement keys (i, j, k, l) OVERRIDE steering angle controls
      Steering angle persists but turn_rate from movement takes precedence
```

## Control Flow Diagram

```
┌─────────────────────────────────────────────────────────┐
│ Keyboard Teleop Layer                                   │
│                                                         │
│ steering_angle (state) ←─ 'a'/'d'/'s'/'h'/'y' keys   │
│ turn_rate (calculated) ← steering_angle / 10.0        │
│                                                         │
│ linear_velocity ←─ 'i'/'j'/'k'/'l'/etc keys           │
│ angular_velocity ←─ turn_rate OR movement ratio       │
└──────────────────┬──────────────────────────────────────┘
                   │ Twist message
                   ▼
┌─────────────────────────────────────────────────────────┐
│ ROS2 micro-ROS Bridge                                   │
│                                                         │
│ /cmd_vel topic: geometry_msgs/msg/Twist               │
│ - linear.x: 0.0 to 2.0 m/s (or -2.0 to 0.0)          │
│ - angular.z: -3.0 to 3.0 rad/s                        │
└──────────────────┬──────────────────────────────────────┘
                   │ Serial transmission
                   ▼
┌─────────────────────────────────────────────────────────┐
│ ESP32 Control Layer (main.cpp)                          │
│                                                         │
│ cmdVelCallback():                                       │
│ - Extract linear.x, angular.z from Twist              │
│ - Convert angular velocity → steering angle            │
│ - Call steeringController.setTargetAngle()            │
│ - Call drivingController.setVelocity()                │
└──────────────────┬──────────────────────────────────────┘
                   │
          ┌────────┴────────┐
          ▼                 ▼
┌──────────────────┐  ┌──────────────────┐
│ Steering Control │  │ Driving Control  │
│                  │  │                  │
│ setTargetAngle() │  │ setVelocity()    │
│ update() w/PID   │  │ update() w/accel │
│ Read encoder     │  │ Smooth ramp      │
│ Control motor    │  │ Control motor    │
└────────┬─────────┘  └────────┬─────────┘
         │                     │
         └─────────┬───────────┘
                   ▼
         ┌─────────────────────┐
         │ Motor Outputs (PWM) │
         │                     │
         │ Pins 25,26 (Steer)  │
         │ Pins 27,14 (Drive)  │
         └─────────────────────┘
```

## Parameter Flow

### Teleop → Twist Message

```python
# Keyboard settings (user adjustable with q/z/w/x/e/c)
speed = 0.5 m/s        (linear velocity scale)
turn = 1.0 rad/s       (angular velocity scale)

# From keyboard input
steering_angle = 4.0°
movement = 'i' → (1, 0)

# Calculate velocities
linear_vel = 1 * speed = 0.5 m/s
angular_vel = 0 * turn = 0.0 rad/s  # Movement overrides steering

# Twist message
twist.linear.x = 0.5 m/s
twist.angular.z = 0.0 rad/s
```

### Twist → ESP32 Control

```cpp
// In ESP32 main.cpp cmdVelCallback()
float linearVel = msg->linear.x;      // 0.5 m/s
float angularVel = msg->angular.z;    // 0.0 rad/s

// Convert angular velocity to steering angle
if (abs(linearVel) > 0.01) {
    steeringAngle = atan(angularVel * WHEELBASE / linearVel) * (180/PI);
} else {
    steeringAngle = (angularVel > 0) ? 10 : -10;
}
// Result: steeringAngle ≈ 0.0° (since angularVel = 0)

// Set targets
steeringController.setTargetAngle(steeringAngle);    // 0.0°
drivingController.setVelocity(linearVel);            // 0.5 m/s
```

## Timing & Update Rates

```
Keyboard Level (Asynchronous):
  └─ User presses key
  └─ Twist message published immediately

ROS2 micro-ROS (10 Hz default):
  └─ /cmd_vel subscription processed
  └─ cmdVelCallback() runs

ESP32 Control Loop (100 Hz):
  ├─ steeringController.update() every 10ms
  │  ├─ Read encoder: ISR every pulse (very fast)
  │  ├─ Compute PID: every 10ms
  │  └─ Set motor PWM: every 10ms
  │
  ├─ drivingController.update() every 10ms
  │  ├─ Smooth acceleration
  │  └─ Set motor PWM
  │
  ├─ publishStatus() every 100ms
  │  └─ Send /ugv/status to ROS2
  │
  └─ updateDisplay() every 200ms
     └─ Update OLED with current state
```

## Example: Complete Maneuver Sequence

**Goal**: Drive forward 1 meter, turn right 45°, continue forward

### Steps

```
1. Reset steering
   Keystroke: 's'
   Effect: steering_angle = 0.0°, move forward straight

2. Increase speed
   Keystroke: 'q'
   Effect: speed *= 1.1 (0.5 → 0.55 m/s)

3. Move forward
   Keystroke: 'i'
   Effect: Twist(linear.x=0.55, angular.z=0.0)
   Result: Robot moves forward straight
   Duration: ~2 seconds (1 meter at 0.55 m/s)

4. Start turning right
   Keystroke: 'd', 'd', 'd', 'd', 'd'
   Effect: steering_angle: 0° → 2° → 4° → 6° → 8° → 10°
   Result: Each keystroke sends Twist with angular.z based on steering_angle
   Behavior: ESP32 converts angle → control signal

5. Continue turning while moving
   Still pressing 'i' (forward still active)
   Effect: Twist(linear.x=0.55, angular.z from movement='i'=0.0)
   Note: Movement keys override steering angle controls

6. Reset steering for straight
   Keystroke: 's'
   Effect: steering_angle = 0.0°

7. Stop
   Keystroke: 'k' or space
   Effect: Twist(linear.x=0.0, angular.z=0.0)
   Result: Robot stops
```

## Troubleshooting Integration

| Issue | Check | Fix |
|-------|-------|-----|
| Keys not working | ROS2 running? | `ros2 node list` |
| Robot not responding | /cmd_vel topic? | `ros2 topic echo /cmd_vel` |
| Steering not moving | ESP32 calibrated? | Run calibration in setup |
| Steering wrong angle | Encoder pulses? | Verify 500 pulses = 10° |
| Jerky movement | PID tuning | Adjust SteeringController.h gains |
| Speed not matching | Linear speed scale | Adjust `speed` parameter in teleop |
| Lag in response | ROS2 latency | Reduce network traffic |

## Command Reference

### Start Teleop
```bash
cd ~/ackerman_ugv_ws
source install/setup.bash
ros2 run ugv_teleop teleop_keyboard
```

### Monitor Performance
```bash
# In another terminal
ros2 topic echo /cmd_vel      # See velocity commands
ros2 topic echo /ugv/status   # See steering feedback
ros2 topic list               # See all available topics
```

### Test Without Robot
```bash
# Publish manual Twist messages
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.2}}'
```
