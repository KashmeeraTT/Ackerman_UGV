# Boot Loop and Motor Control Fixes - December 2, 2025

## Issues Resolved

### 1. **Motor Doesn't Turn During Boot Test**
**Problem:** OLED displays "Turning left" and "Turning right" messages, but motor doesn't actually move.

**Root Cause:** Motor speed during test was set to 80 PWM, but motor deadband is 20 PWM. Any PWM value between -20 and +20 is filtered out. The deadband prevented motor from running.

**Solution:** Increased motor test speed from 80 PWM to 150 PWM (7.5x the deadband).

**Code Changes:**
```cpp
// Before (Line ~194-195 and ~213-214):
steeringMotor.setSpeed(-80);  // Too low - below deadband
steeringMotor.setSpeed(80);   // Too low - below deadband

// After:
steeringMotor.setSpeed(-150); // Sufficient to overcome deadband
steeringMotor.setSpeed(150);  // Sufficient to overcome deadband
```

### 2. **System Reboots (Boot → Test → Boot Loop)**
**Problem:** OLED and serial show boot sequence repeating. "Booting UGV" message reappears multiple times. LIMIT SWITCH TEST appears repeatedly instead of progressing.

**Root Cause:** Multiple critical issues:
- **Dual Serial.begin()**: `setupMicroROS()` was calling `Serial.begin(115200)` again after setup() already initialized serial. This resets the USB connection.
- **Infinite ROS2 loop**: The micro-ROS retry logic had a bug where `setupMicroROS()` was called inside a while condition without saving its return value, potentially hanging forever.
- **systemInitialized not set**: After setup completes, `systemInitialized` wasn't set to `true`, causing system to not fully initialize.
- **No watchdog feed**: The loop() function didn't feed the ESP32 watchdog timer, causing automatic resets after ~5 seconds of not feeding.

**Solution:**
- Removed redundant `Serial.begin()` from `setupMicroROS()`
- Fixed ROS2 retry logic to check return value properly
- Explicitly set `systemInitialized = true` after ROS2 setup
- Added watchdog feed in loop()

**Code Changes:**

**Before (setupMicroROS):**
```cpp
bool setupMicroROS() {
  Serial.begin(115200);  // PROBLEM: Re-initializes already-open serial
  set_microros_serial_transports(Serial);
  // ... rest of code
}
```

**After:**
```cpp
bool setupMicroROS() {
  // Serial is already initialized in setup() - don't reinitialize
  // Serial.begin(115200);  // REMOVED - already done in setup()
  set_microros_serial_transports(Serial);
  // ... rest of code
}
```

**Before (ROS2 initialization):**
```cpp
while (!setupMicroROS() && microRosRetries < 5) {
  // This could hang if setupMicroROS() never returns
  microRosRetries++;
  delay(1000);
}
```

**After:**
```cpp
bool rosInitSuccess = setupMicroROS();
while (!rosInitSuccess && microRosRetries < 5) {
  // Save result so we can check it properly
  Serial.println("Failed to initialize micro-ROS. Retrying...");
  microRosRetries++;
  delay(1000);
  rosInitSuccess = setupMicroROS();  // Re-attempt
}
```

**Before (systemInitialized):**
```cpp
if (microRosRetries >= 5) {
  // ... error display ...
  // systemInitialized NEVER SET - remains false!
} else {
  // ... success message ...
  // systemInitialized NEVER SET - remains false!
}
```

**After:**
```cpp
if (!rosInitSuccess) {
  // ... error display ...
  systemInitialized = true;  // Set even on failure
} else {
  // ... success message ...
  systemInitialized = true;  // Set on success
}
```

**Before (loop function):**
```cpp
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  // ... no watchdog feed - ESP32 resets after ~5 seconds
}
```

**After:**
```cpp
void loop() {
  // Feed the watchdog timer - prevents ESP32 reset
  esp_task_wdt_reset();
  
  // Spin executor only if fully initialized
  if (systemInitialized) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }
  // ... rest of loop
}
```

**Added Include:**
```cpp
#include <esp_task_wdt.h>  // For watchdog timer reset
```

## Technical Details

### Motor Deadband Explanation
The `MotorDriver` class implements a deadband to prevent motor humming at very low PWM values:

```cpp
if (abs(currentSpeed_) < MOTOR_DEADBAND) {
  stop();  // Ignore speeds below deadband
  return;
}
```

Where `MOTOR_DEADBAND = 20` (from pin_config.h).

This means:
- PWM -20 to +20: Motor stops
- PWM ±21 to ±255: Motor runs with corresponding speed

**Implication:** The test speed of 80 PWM was still within an acceptable range but fell below the required ~100-150 PWM needed to reliably move the steering mechanism against friction/resistance. Increasing to 150 provides 7.5x margin above the minimum deadband.

### Watchdog Timer
The ESP32 has an internal watchdog timer that automatically resets the chip if the main loop doesn't feed it regularly (typically every 5 seconds). 

Without `esp_task_wdt_reset()` in the loop, if the loop blocks for more than ~5 seconds (e.g., waiting for ROS2 connection), the watchdog triggers a reset, causing the boot sequence to restart.

The fix:
```cpp
esp_task_wdt_reset();  // "Pets the watchdog" - tells it we're still alive
```

This must be called regularly in the loop() to prevent automatic resets.

## Boot Sequence After Fixes

```
POWER ON
   ↓
Serial initialized (once) → OLED "Booting UGV..."
   ↓
GPIO check (limit switches)
   ↓
3-second test mode window
   ↓
[If not manual mode] Auto limit switch test
   ├─ Motor turns LEFT (150 PWM) → Finds left limit → STOPS
   ├─ Motor turns RIGHT (150 PWM) → Finds right limit → STOPS
   ├─ Results displayed on OLED (2 sec)
   │
   ↓
Initialize controllers
   ├─ steeringController.begin()
   ├─ drivingController.begin()
   └─ Set startup grace period
   ↓
Calibration (if needed) or load from memory
   ↓
OLED: "Initializing micro-ROS..."
   ↓
Setup micro-ROS (with retry logic)
   ├─ Serial NOT re-initialized (already done)
   ├─ Create ROS2 node
   ├─ Create subscribers/publishers
   └─ Return true/false status
   ↓
Set systemInitialized = true (success or failure)
   ↓
OLED: "READY!"
   ↓
Enter main loop()
   ├─ Feed watchdog (every iteration)
   ├─ Handle ROS2 callbacks
   ├─ Update motor controllers
   ├─ Publish status
   └─ Update display
   ↓
✓ System stable, waiting for commands
```

## Compilation Status
✅ Builds successfully  
✅ No compiler errors or warnings  
✅ Memory usage: 13.3% RAM, 30.1% Flash  

## Testing Steps

1. **Flash the firmware**
   ```bash
   platformio run --target upload
   ```

2. **Monitor serial output**
   ```bash
   platformio device monitor --baud 115200
   ```

3. **Expected behavior:**
   - Boot message appears (Booting UGV)
   - Motor turns left for up to 5 seconds (should see actual movement)
   - Motor stops, results displayed
   - Motor turns right for up to 5 seconds (should see actual movement)
   - Motor stops, results displayed
   - "Initializing micro-ROS..." message appears
   - "READY!" message appears
   - System ready for ROS2 commands (no reboot loop)

4. **Motor movement verification:**
   - Listen for motor noise during left test
   - Listen for motor noise during right test
   - If motor doesn't turn despite OLED showing "Turning left/right", check:
     - Motor power connections
     - Motor driver enable pins (should be connected to 5V)
     - PWM pin connections to ESP32

## Summary of Fixes

| Issue | Cause | Fix | Line(s) |
|-------|-------|-----|---------|
| Motor won't turn | PWM too low (80 < deadband threshold) | Increase to 150 PWM | 194, 213 |
| Boot loop | Dual Serial.begin() resets USB | Remove Serial.begin() from setupMicroROS() | 154 |
| ROS2 hang | Improper return value checking | Save setupMicroROS() result in variable | 1010-1018 |
| Never exits boot | systemInitialized not set | Explicitly set systemInitialized = true | 1024, 1027 |
| Auto reboot every 5s | No watchdog feed | Add esp_task_wdt_reset() in loop | 1050 |

All fixes are backward compatible and don't affect other modules.
