// ============================================================================
// UGV ESP32 Firmware - Main Entry Point
// ============================================================================
// Ackermann UGV motor control with micro-ROS integration
// Rewritten with modular architecture
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>

// Configuration
#include "config.h"

// Hardware abstraction
#include "hal/encoder.h"
#include "hal/limit_switch.h"

// Drivers
#include "drivers/bts7960.h"
#include "drivers/display.h"
#include "drivers/imu.h"

// Controllers
#include "controllers/drive_controller.h"
#include "controllers/steering_controller.h"

// Safety
#include "safety/safety_monitor.h"

// Communication
#include "comms/ros_bridge.h"

// ============================================================================
// Global Objects
// ============================================================================

// Motor drivers
BTS7960 steeringMotor(PIN_STEERING_LPWM, PIN_STEERING_RPWM, PWM_CH_STEERING_L,
                      PWM_CH_STEERING_R);
BTS7960 drivingMotor(PIN_DRIVING_LPWM, PIN_DRIVING_RPWM, PWM_CH_DRIVING_L,
                     PWM_CH_DRIVING_R);

// Encoder
Encoder steeringEncoder(PIN_ENCODER_A, PIN_ENCODER_B);

// Limit switches
LimitSwitch leftLimit(PIN_LIMIT_LEFT);
LimitSwitch rightLimit(PIN_LIMIT_RIGHT);

// Sensors
IMU imu;
Display oled;

// Controllers
SteeringController steeringCtrl(steeringMotor, steeringEncoder, leftLimit,
                                rightLimit);
DriveController driveCtrl(drivingMotor);

// Safety
SafetyMonitor safety(steeringCtrl, driveCtrl);

// Communication
RosBridge ros;

// ============================================================================
// State Variables
// ============================================================================

bool systemReady = false;
bool imuAvailable = false;

// Timing
unsigned long lastLoopTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastSteeringPubTime = 0;
unsigned long lastImuTime = 0;
unsigned long lastDisplayTime = 0;

// ============================================================================
// ISR Wrappers (must be static/global)
// ============================================================================

void IRAM_ATTR encoderISR() { steeringEncoder.handleInterrupt(); }

void IRAM_ATTR leftLimitISR() { leftLimit.handleInterrupt(); }

void IRAM_ATTR rightLimitISR() { rightLimit.handleInterrupt(); }

// ============================================================================
// ROS Callback
// ============================================================================

void cmdVelCallback(float linearX, float angularZ) {
  safety.commandReceived();

  if (safety.isEmergencyStop()) {
    return;
  }

  // Set driving velocity
  driveCtrl.setVelocity(linearX);

  // Convert angular velocity to steering angle
  float steeringAngle = 0.0f;
  if (abs(linearX) > 0.01f) {
    steeringAngle = atan(angularZ * WHEELBASE_M / linearX) * (180.0f / PI);
  } else if (abs(angularZ) > 0.01f) {
    steeringAngle =
        (angularZ > 0) ? MAX_STEERING_ANGLE_DEG : MIN_STEERING_ANGLE_DEG;
  }

  steeringCtrl.setTargetAngle(steeringAngle);
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(MICROROS_SERIAL_BAUD);

  // Initialize I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Initialize display first for feedback
  if (oled.begin()) {
    oled.showBoot("Initializing...");
  }
  delay(500);

  Serial.println("\n========================================");
  Serial.println("UGV ESP32 Firmware Starting...");
  Serial.println("========================================\n");

  // Initialize motor drivers
  Serial.println("Initializing motors...");
  steeringMotor.begin();
  drivingMotor.begin();

  // Initialize encoder
  Serial.println("Initializing encoder...");
  steeringEncoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR,
                  CHANGE); // Full 4x resolution

  // Initialize limit switches
  Serial.println("Initializing limit switches...");
  leftLimit.begin();
  rightLimit.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_LEFT), leftLimitISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_RIGHT), rightLimitISR,
                  FALLING);

  // Check limit switches at startup
  bool leftPressed = leftLimit.isPressed();
  bool rightPressed = rightLimit.isPressed();
  Serial.print("Left limit: ");
  Serial.println(leftPressed ? "PRESSED" : "OK");
  Serial.print("Right limit: ");
  Serial.println(rightPressed ? "PRESSED" : "OK");

  // Initialize IMU
  Serial.println("Initializing IMU...");
  imuAvailable = imu.begin();
  if (!imuAvailable) {
    Serial.println("IMU not found - continuing without IMU");
  }

  // Initialize controllers
  Serial.println("Initializing controllers...");
  steeringCtrl.begin();
  steeringCtrl.setDisplay(&oled); // Enable visual calibration feedback
  driveCtrl.begin();

  // Initialize micro-ROS FIRST - so ROS system knows we're alive
  Serial.println("\nInitializing micro-ROS...");
  oled.showBoot("micro-ROS init...");

  int retries = 0;
  while (!ros.begin(cmdVelCallback) && retries < 5) {
    Serial.println("micro-ROS init failed, retrying...");
    retries++;
    delay(1000);
  }

  if (!ros.isConnected()) {
    Serial.println("micro-ROS failed after 5 attempts - continuing anyway");
    oled.showBoot("ROS failed, continuing...");
    delay(1000);
  } else {
    Serial.println("micro-ROS connected!");
  }

  // Set grace period before calibration
  safety.setGracePeriod(true);

  // Calibration
  bool skipCalibration = (leftPressed && rightPressed);
  if (skipCalibration) {
    Serial.println("\n*** TEST MODE: Skipping calibration ***");
    systemReady = true;
  } else {
    Serial.println("\nStarting calibration...");
    oled.showCalibration("Starting...", false, false, 0);

    if (steeringCtrl.calibrate()) {
      Serial.println("Calibration complete!");
      steeringCtrl.setTargetAngle(0);

      // Wait for centering
      unsigned long start = millis();
      while (abs(steeringCtrl.getCurrentAngle()) > 1.0f &&
             (millis() - start < 3000)) {
        steeringCtrl.update();
        delay(10);
      }

      systemReady = true;
    } else {
      Serial.println("Calibration FAILED!");
      oled.showError("Calibration failed");
      delay(3000);
    }
  }

  // Clear limit flags after calibration
  leftLimit.clearTriggered();
  rightLimit.clearTriggered();

  // System ready
  systemReady = true;
  Serial.println("\n========================================");
  Serial.println("System ready!");
  Serial.println("========================================\n");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  esp_task_wdt_reset();

  unsigned long now = millis();

  // Check ROS connection status (pings agent every 2 seconds)
  ros.checkConnection();

  // Spin ROS executor
  if (ros.isConnected()) {
    ros.spin();
  }

  // Main control loop (100 Hz)
  if (now - lastLoopTime >= LOOP_PERIOD_MS) {
    lastLoopTime = now;

    // Safety checks
    safety.update();

    // Update controllers if safe
    if (systemReady && !safety.isEmergencyStop() && !safety.isTimeout()) {
      steeringCtrl.update();
      driveCtrl.update();
    }
  }

  // Heartbeat publishing (5 Hz)
  if (now - lastHeartbeatTime >= HEARTBEAT_PERIOD_MS) {
    lastHeartbeatTime = now;
    ros.publishHeartbeat();
  }

  // Steering angle publishing (20 Hz)
  if (now - lastSteeringPubTime >= STEERING_ANGLE_PERIOD_MS) {
    lastSteeringPubTime = now;
    ros.publishSteeringAngle(steeringCtrl.getCurrentAngle());
  }

  // IMU update and publishing (50 Hz)
  if (imuAvailable && (now - lastImuTime >= IMU_PERIOD_MS)) {
    lastImuTime = now;
    imu.update();

    float qw, qx, qy, qz;
    imu.getQuaternion(qw, qx, qy, qz);
    ros.publishImu(qw, qx, qy, qz, imu.getGyroX(), imu.getGyroY(),
                   imu.getGyroZ(), imu.getAccelX(), imu.getAccelY(),
                   imu.getAccelZ());
  }

  // Status publishing (5 Hz)
  if (now - lastStatusTime >= STATUS_PERIOD_MS) {
    lastStatusTime = now;

    char status[128];
    snprintf(status, sizeof(status),
             "Angle:%.2f Target:%.2f Vel:%.2f Enc:%ld EStop:%d",
             steeringCtrl.getCurrentAngle(), steeringCtrl.getTargetAngle(),
             driveCtrl.getCurrentVelocity(), steeringCtrl.getEncoderCount(),
             safety.isEmergencyStop() ? 1 : 0);
    ros.publishStatus(status);
  }

  // Display update (5 Hz)
  if (oled.isAvailable() && (now - lastDisplayTime >= DISPLAY_PERIOD_MS)) {
    lastDisplayTime = now;

    const char *mode;
    const char *statusCode;

    if (safety.isEmergencyStop()) {
      mode = "E-STOP";
      statusCode = "[STOP]";
    } else if (safety.isTimeout()) {
      mode = "TIMEOUT";
      statusCode = "[TOUT]";
    } else if (!systemReady) {
      mode = "INIT...";
      statusCode = "[INIT]";
    } else if (safety.getCommandAge() < 500) {
      mode = "RUNNING";
      statusCode = "[ GO ]";
    } else {
      mode = "READY";
      statusCode = "[IDLE]";
    }

    oled.showStatus(
        mode, ros.isConnected(), safety.getCommandAge(),
        steeringCtrl.getCurrentAngle(), steeringCtrl.getTargetAngle(),
        driveCtrl.getCurrentVelocity(), steeringCtrl.getEncoderCount(),
        steeringCtrl.getLeftLimitState(), steeringCtrl.getRightLimitState(),
        statusCode);
  }

  // Handle serial reset command
  if (Serial.available() > 0) {
    char c = Serial.read();
    if ((c == 'r' || c == 'R') && safety.isEmergencyStop()) {
      Serial.println("Manual reset...");
      safety.tryResetEmergencyStop();
    }
  }
}