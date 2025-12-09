# UGV Teleop Control System

A complete ROS2 Humble + micro-ROS system for controlling an Ackerman-steering UGV with ESP32.

## System Overview

### Hardware Components
- **ESP32 DOIT DevKit V1**: Low-level motor control and sensor reading
- **BTS7960 Motor Drivers (x2)**: One for steering, one for driving
- **Encoder Motor**: Steering motor with 600 PPM (50 pulses/degree)
- **Limit Switches (x2)**: Safety limits for steering rack (±10°)
- **Differential Drive**: Main driving system (no encoders)

### Software Architecture

```
┌─────────────────────────────────────────────┐
│           ROS2 Humble (Ubuntu 22.04)        │
│  ┌──────────────────┬────────────────────┐  │
│  │  Teleop Keyboard │  micro-ROS Agent   │  │
│  └────────┬─────────┴──────────┬─────────┘  │
│           │     /cmd_vel       │ Serial     │
│           └────────────────────┘            │
└─────────────────────────────────────────────┘
                      │ USB Serial
┌─────────────────────────────────────────────┐
│               ESP32 (micro-ROS)             │
│  ┌──────────────┬────────────┬────────────┐ │
│  │  Steering    │  Driving   │ Calibration│ │
│  │  Controller  │  Controller│  System    │ │
│  └──────┬───────┴─────┬──────┴─────┬──────┘ │
│         │  PID        │  Velocity  │  Auto  │
└─────────┼─────────────┼────────────┼────────┘
          │             │            │
    ┌─────┴─────┐  ┌────┴────┐  ┌───┴────┐
    │ Encoder   │  │ Driving │  │ Limit  │
    │ + Steering│  │  Motor  │  │Switches│
    │   Motor   │  │         │  │        │
    └───────────┘  └─────────┘  └────────┘
```

## Pin Configuration

| Component | Pin | GPIO |
|-----------|-----|------|
| Steering LPWM | GPIO 25 | PWM Channel 0 |
| Steering RPWM | GPIO 26 | PWM Channel 1 |
| Driving LPWM | GPIO 27 | PWM Channel 2 |
| Driving RPWM | GPIO 14 | PWM Channel 3 |
| Encoder A | GPIO 34 | Input only |
| Encoder B | GPIO 35 | Input only |
| Limit Left | GPIO 32 | Interrupt |
| Limit Right | GPIO 33 | Interrupt |

## Installation

### 1. ESP32 Firmware (PlatformIO)

```bash
# Install PlatformIO dependencies (will download micro-ROS)
cd /path/to/Ackerman_UGV
pio pkg install

# Build firmware
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

### 2. ROS2 Workspace & micro-ROS Agent

#### Option A: Build micro-ROS Agent from Source (Recommended)

```bash
# Create micro-ROS workspace
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws

# Clone dependencies and agent
git clone https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ros-agent

# Build
source /opt/ros/humble/setup.bash
colcon build

# Add to bashrc for automatic loading (optional but recommended)
echo 'source ~/micro_ros_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

#### Option B: Install from Package (if available)

```bash
# This may not work depending on your system, but worth trying
sudo apt update
sudo apt install ros-humble-micro-ros-agent
```

#### Build ROS2 Teleop Workspace

```bash
# Build ROS2 workspace
cd /path/to/Ackerman_UGV/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### Quick Start

1. **Connect ESP32** to your laptop via USB

2. **Find serial port**:
   ```bash
   ls /dev/ttyUSB*  # or /dev/ttyACM*
   ```

3. **Update launch file** if needed (default is `/dev/ttyUSB0`):
   Edit `ros2_ws/src/ugv_teleop/launch/ugv_teleop.launch.py`

4. **Launch the system**:
   ```bash
   cd ros2_ws
   source install/setup.bash
   ros2 launch ugv_teleop ugv_teleop.launch.py
   ```

5. **Control the UGV** using keyboard:
   - `i` = Forward
   - `k` = Stop
   - `,` = Backward
   - `j` = Turn left
   - `l` = Turn right
   - `u/o/m/.` = Diagonal movements
   - `space` = Emergency stop
   - `q/z` = Increase/decrease max speeds

### Manual Launch (Individual Components)

**Terminal 1** - micro-ROS Agent:
```bash
# Make sure micro-ROS workspace is sourced (or add to ~/.bashrc)
source ~/micro_ros_ws/install/setup.bash

# Start the agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Terminal 2** - Teleop Keyboard:
```bash
# Source the UGV ROS2 workspace
source /path/to/Ackerman_UGV/ros2_ws/install/setup.bash

# Run teleop
ros2 run ugv_teleop teleop_keyboard
```

## Features

### ESP32 Firmware

- **Automatic Calibration**: Runs on first boot, finds steering center using limit switches
- **Persistent Memory**: Saves calibration to EEPROM/Preferences
- **PID Control**: Smooth steering angle control with encoder feedback
- **Safety Limits**: Hard ±10° steering limits to protect rack
- **Emergency Stop**: Triggered by limit switches or angle violations
- **100Hz Control Loop**: Fast, responsive motor control
- **10Hz Status Publishing**: Real-time status feedback to ROS2

### ROS2 System

- **cmd_vel Interface**: Standard Twist messages for compatibility with NAV2
- **Ackerman Conversion**: Automatic conversion from Twist to steering angle
- **Keyboard Teleop**: Intuitive WASD-style controls
- **Status Monitoring**: Real-time feedback on `/ugv/status` topic

## Calibration Process

1. **Power on UGV**
2. **Automatic limit switch test runs** (tests both switches, ~10 seconds)
3. ESP32 checks for saved calibration in memory
4. If not found:
   - Moves steering **LEFT until limit switch triggers** (NO TIMEOUT - waits indefinitely)
   - Records encoder count at left position
   - Moves steering **RIGHT until limit switch triggers** (NO TIMEOUT - waits indefinitely)
   - Records encoder count at right position
   - Calculates center position: `center = (left_count + right_count) / 2`
   - Moves steering to center position based on encoder values
   - Resets encoder count to 0 at center
   - Saves calibration to ESP32 memory (Preferences)
5. Future boots load calibration from memory (no movement needed)

**Important Notes**:
- ✓ No timeout during calibration - waits for actual limit switch press
- ✓ If already at a limit (e.g., left limit reached), motor won't move further
- ✓ Steering centers perfectly based on encoder reading
- ✓ Calibration is stored and reused on all subsequent boots

## Testing Process

### Phase 1: Hardware Verification

**1.1 Limit Switch Testing** (Before first calibration)

```bash
# Upload firmware and open serial monitor
pio device monitor

# At startup, you'll see:
# - "STARTUP LIMIT SWITCH CHECK"
# - Left Limit Switch (Pin 32): HIGH (Not Pressed)
# - Right Limit Switch (Pin 33): HIGH (Not Pressed)
# - "Send 't' within 3 seconds to enter limit switch test mode..."

# Send 't' to enter test mode:
# - Physically move steering rack LEFT until first limit switch triggers
# - Watch OLED display: "L: PRESS" should appear
# - Physically move steering rack RIGHT until second limit switch triggers
# - Watch OLED display: "R: PRESS" should appear
# - Send 'q' to exit test mode
```

**Expected Output**:
```
✓ Both switches working correctly!
```

⚠️ **Limit Switch Connection Reminder**:
- Switches must be connected to **GND**, not 5V
- When pressed: Pin reads **LOW** (connected to GND)
- When not pressed: Pin reads **HIGH** (floating/pullup from encoder power)

**1.2 Encoder Verification** (During calibration)

```bash
# Encoder is tested automatically during calibration
# Watch serial output:
# - "Starting steering calibration..."
# - OLED shows: "CALIBRATION / Starting..."
# - Motor moves LEFT until limit switch triggers
# - OLED shows: "CALIBRATION / Encoder: [count]"
# - Motor moves RIGHT to opposite limit
# - Calculates center position
# - Motor centers and locks calibration
```

**Expected Output**:
```
Calibration successful!
```

### Phase 2: Initialization & OLED Display

**2.1 Boot Sequence** (Watch OLED and Serial)

```bash
# Power on ESP32
# Serial output should show:
# ========================================
# UGV ESP32 Controller Starting...
# STARTUP LIMIT SWITCH CHECK
# ========================================
# Left Limit Switch (Pin 32, GND-connected): HIGH (Not Pressed)
# Right Limit Switch (Pin 33, GND-connected): HIGH (Not Pressed)
# ✓ Limit switches OK (GND-connected)
```

**OLED Display Sequence**:
1. **Boot Phase**: Shows "Booting UGV..." (briefly)
2. **Calibration Phase** (if needed):
   - Shows "CALIBRATION / Starting..."
   - Then "CALIBRATION / SUCCESS!" (after 1-2 seconds)
   - Displays limit switch states and encoder count
3. **Ready Phase**: Shows "System Ready!" for 2 seconds
   - Followed by "Waiting for ROS2 commands..."
4. **Idle State**: Normal status display
   - Current steering angle / target angle
   - Current velocity
   - Limit switch status (L: OK or L: HIT)
   - ROS2 connectivity status

### Phase 3: ROS2 Connection Testing

**3.1 Start micro-ROS Agent** (Terminal 1)

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Expected Output**:
```
[INFO] [1734773400.123456] [rclc]: Initializing RCL
[INFO] [1734773400.234567] [rclc]: Initialized from ROS2
[INFO] [1734773400.345678] [rclc]: Creating executor
```

**Serial Monitor Shows**:
```
micro-ROS initialized successfully!
System ready!
```

**3.2 Check ROS2 Topics** (Terminal 2)

```bash
# List all topics
ros2 topic list

# Expected output:
/cmd_vel
/parameter_events
/rosout
/ugv/status
```

**3.3 Monitor Status Topic**

```bash
ros2 topic echo /ugv/status

# Expected output (repeats every 100ms):
data: "Angle:0.00 Target:0.00 Vel:0.00 Enc:0 E-Stop:0"
```

### Phase 4: Teleop Command Reception

**4.1 Launch Full Teleop System** (Terminal 3)

```bash
cd ros2_ws
source install/setup.bash
ros2 launch ugv_teleop ugv_teleop.launch.py
```

**Expected Output**:
```
[INFO] [teleop_keyboard-N]: Teleop Keyboard Node started
[INFO] [teleop_keyboard-N]: Use WASD for movement, SPACE for stop
```

**4.2 Test Command Reception with Keyboard**

Send commands using the teleop keyboard:

```
Available Controls:
i = Forward          j = Turn Left      u = Forward-Left
k = Stop             l = Turn Right     o = Forward-Right
, = Backward         . = Backward-Right m = Backward-Left
space = Emergency Stop
q/z = Increase/Decrease Max Speed
```

**What to Watch On OLED**:
1. **Normal State** (before command):
   - Shows "UGV STATUS"
   - Angle values should be stable
   - Vel: 0.00 (no movement)
   - L: OK and R: OK (limit switches not triggered)
   - ROS2: WAITING... (no commands yet)

2. **After Sending Forward Command** (i key):
   - OLED immediately displays: "CMD RECEIVED!"
   - Shows:
     - Linear: 0.50 m/s (or configured speed)
     - Angular: 0.00 rad/s
     - Age: 0-100ms (time since command received)
     - Target Angle: 0.0 deg (since going straight)
   - Display for 1 second, then back to status

3. **After Sending Turn Left Command** (j key):
   - OLED shows: "CMD RECEIVED!"
   - Shows:
     - Linear: 0.00 m/s (turning in place)
     - Angular: 1.00 rad/s (left turn)
     - Target Angle: ~5.0 deg (steering left)
   - Steering motor moves left
   - Angle value changes in status display

4. **After Sending Diagonal Command** (u key):
   - OLED shows: "CMD RECEIVED!"
   - Shows:
     - Linear: 0.50 m/s
     - Angular: 1.00 rad/s
     - Target Angle: ~5.0 deg
   - Both motors engage (steering + driving)

**4.3 Manual Command Publishing** (For detailed testing)

```bash
# Test forward motion
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once

# Test left turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}" --once

# Test forward + left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 1.0}}" --once

# Test backward + right
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}, angular: {z: -0.5}}" --once
```

**Expected Behavior**:
- ESP32 receives each command
- Motors respond appropriately
- OLED displays command details for 1 second
- Status topic shows updated angle/velocity values

### Phase 5: Full Integration Test

**5.1 Complete System Operation**

```bash
# Terminal 1: micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Teleop Keyboard
ros2 run ugv_teleop teleop_keyboard

# Terminal 3: Monitor status
ros2 topic echo /ugv/status

# Terminal 4: Monitor received commands
ros2 topic echo /cmd_vel

# Terminal 5: Serial monitor
pio device monitor
```

**Test Sequence**:

1. **Forward Motion**
   - Press 'i' on keyboard
   - Watch all outputs:
     - OLED: Shows "CMD RECEIVED!" with values
     - Serial: Shows motor speed changes
     - Status topic: Vel increases, Angle stays 0
   - Press 'k' to stop
   - All values return to zero

2. **Steering Left**
   - Press 'j' on keyboard
   - Watch:
     - OLED: Shows "CMD RECEIVED!" with left turn values
     - Serial: Steering motor moves left
     - Status topic: Angle goes negative (left)
   - Press 'k' to stop
   - Steering centers

3. **Complex Motion**
   - Press 'u' (forward + left)
   - Watch both motors:
     - Driving motor accelerates forward
     - Steering motor moves left
     - OLED shows both commands
   - Press 'k' to stop

4. **Limit Switch Safety**
   - Continue turning left with multiple 'j' presses
   - At ±10°, steering should stop responding
   - If limit switch triggered: OLED shows "EMERGENCY STOP"
   - All motion halts

### Phase 6: Performance Metrics

**6.1 Check These Values from Status Topic**

```bash
ros2 topic echo /ugv/status --once
# data: "Angle:0.00 Target:5.00 Vel:0.50 Enc:250 E-Stop:0"
```

- **Angle**: Current steering angle (should track Target)
- **Target**: Commanded angle (0 = straight, ±10 = max)
- **Vel**: Current driving velocity (0.0 to 1.0 m/s)
- **Enc**: Encoder count (±500 = ±10°)
- **E-Stop**: 0 = normal, 1 = emergency stop active

**6.2 Check OLED Update Rate**

- Status display should update every 200ms (5Hz)
- Command display should appear within 100ms of command
- No visual flicker or lag

**6.3 Check Connectivity**

From Status Display on OLED, ROS2 line shows:
- `ROS2: CONNECTED` - Command received within last 2 seconds
- `ROS2: IDLE (5s)` - No command for 5 seconds
- `ROS2: WAITING...` - No command ever received

## Monitoring & Debugging

### View ROS2 Topics

```bash
# List all topics
ros2 topic list

# Monitor cmd_vel commands
ros2 topic echo /cmd_vel

# Monitor UGV status
ros2 topic echo /ugv/status

# Test cmd_vel manually
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### ESP32 Serial Monitor

```bash
pio device monitor
```

Expected output:
```
UGV ESP32 Controller Starting...
Calibration loaded from memory.
Initializing micro-ROS...
micro-ROS initialized successfully!
System ready!
```

## Troubleshooting

### Emergency Stop Activated (No Reboot Required!)

**Symptoms**: 
- OLED shows "EMERGENCY STOP"
- Serial shows "⚠ EMERGENCY STOP: Limit switch activated!"
- System accepts commands but doesn't execute them

**Recovery (WITHOUT reboot)**:

1. **Check Physical Position**
   - Look at steering rack position
   - Verify it's not actually at a limit switch

2. **Reset via Serial Command**
   ```bash
   # Open serial monitor
   pio device monitor
   
   # When you see "Emergency stop" message:
   # - If steering is NOT at limit: Send 'r' to reset
   # - If steering IS at limit: Manually move steering away first, then send 'r'
   ```

3. **What Happens When You Send 'r'**:
   - Serial shows: "✓ Emergency stop cleared. System resumed."
   - OLED shows: "RESET OK / Ready for commands"
   - System resumes normal operation
   - No reboot needed!

4. **Example Serial Flow**:
   ```
   ⚠ EMERGENCY STOP: Limit switch activated!
   Current steering angle: 9.87°
   Send 'r' to reset emergency stop after moving steering away from limit.
   
   [User manually moves steering back to center]
   
   [User sends 'r' via serial monitor]
   
   Manually resetting emergency stop...
   ✓ Emergency stop reset. System ready.
   ```

**Why This Happens**:
- Limit switches are safety features
- They detect when steering reaches maximum rotation (±10°)
- Sometimes spurious triggers can occur during micro-ROS connection
- System prevents accidental damage by stopping all motion

**Prevention**:
- Ensure steering is centered before power-on
- Check limit switch wiring is secure
- Use 'r' command quickly to reset and resume operation

### micro-ROS Agent Connection Failed

**Symptoms**: "Failed to initialize micro-ROS. Retrying..."

**Solutions**:
1. Check USB cable connection
2. Verify serial port: `ls /dev/ttyUSB*`
3. Ensure micro-ROS agent is running
4. Check baud rate (must be 115200)
5. Try resetting ESP32

### Limit Switch Triggered

**Symptoms**: "EMERGENCY STOP: Limit switch activated!"

**Quick Recovery Steps**:
1. Manually move steering away from the limit (toward center)
2. Open serial monitor (`pio device monitor`)
3. Send 'r' (just type 'r' and press Enter)
4. System resets without reboot!

**If Problem Persists**:
1. Check limit switch wiring
2. Verify switches are connected to GND (not 5V)
3. Check for bent or damaged steering rack
4. Test switches manually in test mode (send 't' at startup)

### Steering Not Responding

**Possible causes**:
- Calibration not complete
- Emergency stop active (send 'r' to reset)
- Encoder wiring issue
- Motor driver connections

**Debug**:
```bash
# Check status topic
ros2 topic echo /ugv/status

# Look for:
# - E-Stop: 0 (should be 0)
# - Angle and Target values changing

# Check serial monitor for detailed info
pio device monitor
```

## Future Extensions

### NAV2 Integration (Planned)

The system is designed for easy NAV2 integration:

1. **Add ZED Camera**
2. **Implement Odometry** (when wheel encoders added)
3. **Create robot_state_publisher** with URDF
4. **Configure NAV2** parameters
5. **Enable Autonomous Navigation**

### File Structure

```
Ackerman_UGV/
├── platformio.ini          # ESP32 build config
├── include/                # ESP32 header files
│   ├── pin_config.h       # GPIO pin assignments
│   ├── MotorDriver.h      # BTS7960 driver class
│   ├── SteeringController.h  # Steering control + PID
│   └── DrivingController.h   # Velocity control
├── src/
│   └── main.cpp           # ESP32 firmware (micro-ROS)
└── ros2_ws/
    └── src/
        └── ugv_teleop/    # ROS2 teleop package
            ├── ugv_teleop/
            │   └── teleop_keyboard.py
            └── launch/
                └── ugv_teleop.launch.py
```

## Safety Notes

⚠️ **CRITICAL**: The steering rack has a maximum rotation of ±10°. Exceeding this **WILL damage the hardware**.

The system has multiple safety layers:
1. Software limits in steering controller
2. Hardware limit switches
3. Emergency stop on switch activation
4. Angle validation in encoder ISR

**Always**:
- Test with vehicle elevated first
- Keep emergency stop accessible
- Monitor serial output during initial tests
- Verify limit switches before operation

## Parameters

### Firmware Constants (pin_config.h)

- `MAX_STEERING_ANGLE_DEG`: 10.0° (hardware limit)
- `ENCODER_PPR`: 600 pulses/revolution
- `PULSES_PER_DEGREE`: 50 pulses/degree
- `PWM_FREQUENCY`: 1000 Hz
- `CONTROL_PERIOD_MS`: 10 ms (100Hz)

### ROS2 Parameters

- `speed`: Max linear velocity (default: 0.5 m/s)
- `turn`: Max angular velocity (default: 1.0 rad/s)

## License

MIT License (or specify your license)

## Author

Thasindu Wickrama

## Version

1.0.0 - Initial Release
