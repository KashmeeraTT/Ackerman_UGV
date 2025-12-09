#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <esp_task_wdt.h>

#include "DrivingController.h"
#include "MotorDriver.h"
#include "SteeringController.h"
#include "pin_config.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ========================================
// Global Objects
// ========================================

// Motor drivers
MotorDriver steeringMotor(STEERING_LPWM_PIN, STEERING_RPWM_PIN,
                          STEERING_LPWM_CHANNEL, STEERING_RPWM_CHANNEL);
MotorDriver drivingMotor(DRIVING_LPWM_PIN, DRIVING_RPWM_PIN,
                         DRIVING_LPWM_CHANNEL, DRIVING_RPWM_CHANNEL);

// Controllers
SteeringController steeringController(steeringMotor);
DrivingController drivingController(drivingMotor);

// micro-ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Subscribers
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// Publishers
rcl_publisher_t status_publisher;
std_msgs__msg__String status_msg;

char status_buffer[256];

// OLED Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
bool oledFound = false;

// Last received command tracking
float lastLinearVel = 0.0;
float lastAngularVel = 0.0;
unsigned long lastCmdVelTime = 0;
bool cmdVelReceived = false;

// Timing
unsigned long lastControlUpdate = 0;
unsigned long lastStatusPublish = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long CONTROL_PERIOD_MS = 10;  // 100Hz control loop
const unsigned long STATUS_PERIOD_MS = 100;  // 10Hz status publishing
const unsigned long DISPLAY_PERIOD_MS = 200; // 5Hz display update

// System state
bool systemInitialized = false;
bool emergencyStopActive = false;
unsigned long systemInitializeTime = 0; // Track when system becomes ready
volatile bool inStartupGracePeriod = false; // Flag to suppress emergency stop during startup
const unsigned long STARTUP_GRACE_PERIOD = 2000; // 2 seconds after micro-ROS init

// ========================================
// Function Declarations
// ========================================

void cmdVelCallback(const void *msgin);
void publishStatus();
void performCalibration();
void updateControllers();
void checkErrorStates();
void testLimitSwitches();
void testSteeringAngle();  // New: Test steering angle control

bool setupMicroROS();
void updateDisplay();
void displayError(const char *error);
void displayCalibrationStatus(const char *message);
void displayCommandReceived();
void displayLimitSwitchStatus();
void displayConnectivityStatus();

// ========================================
// micro-ROS Callback
// ========================================

/**
 * @brief Callback for cmd_vel topic
 * Converts Twist message to Ackerman steering control
 */
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  // Track command reception
  lastLinearVel = msg->linear.x;
  lastAngularVel = msg->angular.z;
  lastCmdVelTime = millis();
  cmdVelReceived = true;

  if (emergencyStopActive) {
    return; // Ignore commands during emergency stop
  }

  // Extract linear and angular velocities
  float linearVel = msg->linear.x;   // m/s
  float angularVel = msg->angular.z; // rad/s

  // Set driving velocity
  drivingController.setVelocity(linearVel);

  // Convert angular velocity to steering angle (simplified Ackerman)
  // For low-speed robots: steering_angle ≈ atan(angular_vel * wheelbase /
  // linear_vel) Using simplified mapping: small angular velocities map to
  // steering angles This assumes wheelbase and kinematic model will be tuned
  // during testing

  float steeringAngle = 0.0;
  if (abs(linearVel) > 0.01) {
    // Estimate steering angle from angular/linear velocity ratio
    // Adjust the scaling factor based on your UGV's geometry
    const float WHEELBASE = 0.4; // meters (adjust to your UGV)
    steeringAngle = atan(angularVel * WHEELBASE / linearVel) * (180.0 / PI);
  } else if (abs(angularVel) > 0.01) {
    // Pure rotation - use maximum steering
    steeringAngle =
        (angularVel > 0) ? MAX_STEERING_ANGLE_DEG : MIN_STEERING_ANGLE_DEG;
  }

  // Apply steering angle
  steeringController.setTargetAngle(steeringAngle);
}

// ========================================
// Setup Functions
// ========================================

/**
 * @brief Setup micro-ROS
 */
bool setupMicroROS() {
  // Serial is already initialized in setup() - don't reinitialize
  // Serial.begin(115200);  // REMOVED - already done in setup()
  set_microros_serial_transports(Serial);

  delay(2000); // Wait for connection

  allocator = rcl_get_default_allocator();

  // Create init options
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Create node
  if (rclc_node_init_default(&node, "ugv_esp32_node", "", &support) !=
      RCL_RET_OK) {
    return false;
  }

  // Create cmd_vel subscriber
  if (rclc_subscription_init_default(
          &cmd_vel_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/cmd_vel") != RCL_RET_OK) {
    return false;
  }

  // Create status publisher
  if (rclc_publisher_init_default(
          &status_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/ugv/status") != RCL_RET_OK) {
    return false;
  }

  // Create executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) !=
      RCL_RET_OK) {
    return false;
  }

  // Add subscription to executor
  if (rclc_executor_add_subscription(&executor, &cmd_vel_subscriber,
                                     &cmd_vel_msg, &cmdVelCallback,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  // Initialize status message
  status_msg.data.data = status_buffer;
  status_msg.data.capacity = sizeof(status_buffer);

  return true;
}

/**
 * @brief Perform automatic limit switch testing at startup
 */
void autoTestLimitSwitches() {
  Serial.println("\n========================================");
  Serial.println("AUTOMATIC LIMIT SWITCH TEST");
  Serial.println("========================================");

  if (oledFound) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("LIMIT SWITCH TEST"));
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    display.setCursor(0, 15);
    display.println(F("Testing Left..."));
    display.display();
  }

  // Test left limit switch
  Serial.println("\nTesting LEFT limit switch...");
  Serial.println("Moving steering motor LEFT");
  Serial.println("Waiting for LEFT limit switch to be pressed...");

  // Move steering motor left
  steeringMotor.setSpeed(-150); // Move left at sufficient speed to overcome deadband
  unsigned long testStart = millis();
  bool leftPressed = false;

  // Wait indefinitely for actual limit switch press (no timeout)
  // User must physically press the switch or move it to the limit
  while (!leftPressed) {
    // Read limit switch state directly
    if (digitalRead(LIMIT_SWITCH_LEFT) == LOW) {
      leftPressed = true;
      Serial.println("✓ LEFT limit switch PRESSED!");
      break;
    }
    
    // Safety: If motor running for too long (>30 sec), abort
    if (millis() - testStart > 30000) {
      Serial.println("✗ LEFT limit switch test timeout (30 seconds)!");
      break;
    }
    
    delay(10);
  }

  steeringMotor.stop();
  delay(500);

  if (!leftPressed) {
    Serial.println("✗ LEFT limit switch NOT triggered!");
  }

  if (oledFound) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("LIMIT SWITCH TEST"));
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    display.setCursor(0, 15);
    display.print(F("L: "));
    display.println(leftPressed ? F("PASS") : F("FAIL"));
    display.println(F("Testing Right..."));
    display.display();
  }

  delay(500);

  // Test right limit switch
  Serial.println("\nTesting RIGHT limit switch...");
  Serial.println("Moving steering motor RIGHT");
  Serial.println("Waiting for RIGHT limit switch to be pressed...");

  // Move steering motor right
  steeringMotor.setSpeed(150); // Move right at sufficient speed to overcome deadband
  testStart = millis();
  bool rightPressed = false;

  // Wait indefinitely for actual limit switch press (no timeout)
  // User must physically press the switch or move it to the limit
  while (!rightPressed) {
    // Read limit switch state directly
    if (digitalRead(LIMIT_SWITCH_RIGHT) == LOW) {
      rightPressed = true;
      Serial.println("✓ RIGHT limit switch PRESSED!");
      break;
    }
    
    // Safety: If motor running for too long (>30 sec), abort
    if (millis() - testStart > 30000) {
      Serial.println("✗ RIGHT limit switch test timeout (30 seconds)!");
      break;
    }
    
    delay(10);
  }

  steeringMotor.stop();
  delay(500);

  if (!rightPressed) {
    Serial.println("✗ RIGHT limit switch NOT triggered!");
  }

  // Display final test results
  if (oledFound) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("LIMIT SWITCH TEST"));
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    display.setCursor(0, 15);
    display.print(F("L: "));
    display.println(leftPressed ? F("PASS") : F("FAIL"));
    display.print(F("R: "));
    display.println(rightPressed ? F("PASS") : F("FAIL"));
    display.setCursor(0, 40);
    if (leftPressed && rightPressed) {
      display.println(F("✓ Both OK!"));
    } else {
      display.println(F("✗ Check wiring"));
    }
    display.display();
  }

  // Test summary
  Serial.println("\n========================================");
  Serial.println("TEST SUMMARY:");
  Serial.print("Left switch: ");
  Serial.println(leftPressed ? "PASS" : "FAIL");
  Serial.print("Right switch: ");
  Serial.println(rightPressed ? "PASS" : "FAIL");
  Serial.println("========================================\n");

  if (leftPressed && rightPressed) {
    Serial.println("✓ Both switches working correctly!");
  } else {
    Serial.println("⚠ Warning: Not all switches working properly.");
    Serial.println("Check switch wiring and connections.");
  }

  delay(2000);  // Show result for 2 seconds instead of 3
}

/**
 * @brief Perform steering calibration with detailed feedback
 */
void performCalibration() {
  Serial.println("\n========================================");
  Serial.println("STARTING STEERING CALIBRATION");
  Serial.println("========================================");
  displayCalibrationStatus("Starting...");

  // Set instance pointer for ISR
  SteeringController::instance_ = &steeringController;

  if (steeringController.calibrate()) {
    Serial.println("\n✓ Calibration successful!");
    Serial.println("Steering motor centered and ready.");
    
    if (oledFound) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("CALIBRATION"));
      display.setCursor(0, 20);
      display.println(F("SUCCESS!"));
      display.setTextSize(1);
      display.setCursor(0, 45);
      display.println(F("Ready for ROS2"));
      display.display();
    }
    
    delay(2000);
    systemInitialized = true;
  } else {
    Serial.println("\n✗ Calibration failed!");
    Serial.println("Check limit switches and encoder.");
    
    if (oledFound) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("CALIBRATION"));
      display.setCursor(0, 20);
      display.println(F("FAILED!"));
      display.setTextSize(1);
      display.setCursor(0, 45);
      display.println(F("Check switches"));
      display.display();
    }
    
    delay(3000);
    systemInitialized = false;
  }
}

// ========================================
// Main Control Functions
// ========================================

/**
 * @brief Update all controllers
 */
void updateControllers() {
  steeringController.update();
  drivingController.update();
}

/**
 * @brief Check for error states
 */
/**
 * @brief Check for error states and handle emergency stop with auto-recovery
 */
void checkErrorStates() {
  // Check if grace period has expired
  if (inStartupGracePeriod && (millis() - systemInitializeTime >= STARTUP_GRACE_PERIOD)) {
    inStartupGracePeriod = false;
    steeringController.setStartupGracePeriod(false); // Tell ISRs to start monitoring again
    Serial.println("⏱ Startup grace period expired. Emergency stop protection active.");
  }
  
  // Attempt to auto-clear emergency stop if steering is in safe zone
  steeringController.attemptAutoClearEmergencyStop();
  
  // Check if emergency stop is active (but ignore during grace period)
  if (steeringController.isEmergencyStop() && !inStartupGracePeriod) {
    if (!emergencyStopActive) {
      emergencyStopActive = true;
      drivingController.stop();
      Serial.println("\n⚠ EMERGENCY STOP: Limit switch activated!");
      Serial.println("Current steering angle: " + String(steeringController.getCurrentAngle(), 2) + "°");
      Serial.println("Move steering away from limit to auto-recover, or send 'r' to manually reset.");
    }
  } else {
    // No emergency stop (or in grace period) - if was previously active, clear it
    if (emergencyStopActive && !inStartupGracePeriod) {
      emergencyStopActive = false;
      Serial.println("\n✓ Emergency stop cleared. System resumed.");
    }
    
    // During grace period, suppress emergency stop state
    if (inStartupGracePeriod && emergencyStopActive) {
      emergencyStopActive = false;
    }
  }

  // Check for reset command from serial (for emergency stop recovery)
  if (Serial.available() > 0) {
    char c = Serial.read();
    if ((c == 'r' || c == 'R') && emergencyStopActive) {
      // Only allow reset if user sends explicit command
      Serial.println("\nManually resetting emergency stop...");
      steeringController.forceResetEmergencyStop();
      emergencyStopActive = false;
      Serial.println("✓ Emergency stop reset. System ready.");
      
      if (oledFound) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 10);
        display.println(F("RESET OK"));
        display.setTextSize(1);
        display.setCursor(0, 35);
        display.println(F("Ready for commands"));
        display.display();
        delay(2000);
      }
    }
  }
}

/**
 * @brief Publish status message
 */
void publishStatus() {
  if (!systemInitialized) {
    return;
  }

  // Format status string
  snprintf(status_buffer, sizeof(status_buffer),
           "Angle:%.2f Target:%.2f Vel:%.2f Enc:%ld E-Stop:%d",
           steeringController.getCurrentAngle(),
           steeringController.getTargetAngle(),
           drivingController.getCurrentVelocity(),
           steeringController.getEncoderCount(), emergencyStopActive ? 1 : 0);

  status_msg.data.size = strlen(status_buffer);

  rcl_ret_t ret = rcl_publish(&status_publisher, &status_msg, NULL);
  if (ret != RCL_RET_OK) {
    // Log error if needed (silent for now to avoid serial spam)
  }
}

// ========================================
// Limit Switch Testing
// ========================================

/**
 * @brief Test steering angle control with encoder feedback
 * Allows manual testing of steering angles and encoder readings
 */
void testSteeringAngle() {
  Serial.println("\n========================================");
  Serial.println("STEERING ANGLE CONTROL TEST");
  Serial.println("========================================");
  Serial.println("Commands:");
  Serial.println("  a 5   - Move to +5 degrees");
  Serial.println("  a -5  - Move to -5 degrees");
  Serial.println("  a 0   - Move to center (0 degrees)");
  Serial.println("  s     - Show current angle");
  Serial.println("  q     - Quit test mode\n");

  unsigned long lastDisplayUpdate = 0;
  float targetAngle = 0.0;

  while (true) {
    // Read serial commands
    if (Serial.available() > 0) {
      char command = Serial.read();
      
      if (command == 'q' || command == 'Q') {
        Serial.println("\nExiting steering test mode...");
        steeringMotor.stop();
        break;
      }
      else if (command == 'a' || command == 'A') {
        // Read angle value
        float angle = Serial.parseFloat();
        // Consume newline
        while (Serial.available() && Serial.read() != '\n') {}
        
        targetAngle = constrain(angle, MIN_STEERING_ANGLE_DEG, MAX_STEERING_ANGLE_DEG);
        steeringController.setTargetAngle(targetAngle);
        
        Serial.print("Set target angle to: ");
        Serial.println(targetAngle, 1);
      }
      else if (command == 's' || command == 'S') {
        // Consume any remaining characters on this line
        while (Serial.available() && Serial.read() != '\n') {}
      }
    }

    // Update steering controller
    steeringController.update();

    // Display update every 200ms
    if (millis() - lastDisplayUpdate >= 200) {
      lastDisplayUpdate = millis();

      // Serial output
      Serial.print("Target: ");
      Serial.print(targetAngle, 1);
      Serial.print(" deg | Current: ");
      Serial.print(steeringController.getCurrentAngle(), 1);
      Serial.print(" deg | Encoder: ");
      Serial.println(steeringController.getEncoderCount());

      // OLED display
      if (oledFound) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);

        display.println(F("STEERING TEST"));
        display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

        display.setCursor(0, 15);
        display.setTextSize(2);

        // Target angle
        display.print(F("Target: "));
        display.println(targetAngle, 1);

        // Current angle
        display.setTextSize(1);
        display.print(F("Curr: "));
        display.println(steeringController.getCurrentAngle(), 1);

        // Encoder count
        display.print(F("Enc: "));
        display.println(steeringController.getEncoderCount());

        // Error
        float error = targetAngle - steeringController.getCurrentAngle();
        display.print(F("Error: "));
        display.println(error, 1);

        display.display();
      }
    }

    delay(10);
  }

  if (oledFound) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Test Complete"));
    display.display();
    delay(1000);
  }
}

// ========================================
// Limit Switch Testing
// ========================================

/**
 * @brief Test limit switches and display their states
 */
void testLimitSwitches() {
  Serial.println("\n========================================");
  Serial.println("LIMIT SWITCH TEST MODE");
  Serial.println("========================================");
  Serial.println("Press each limit switch to test.");
  Serial.println("Both switches should read HIGH when not pressed.");
  Serial.println("Press 'q' in Serial Monitor to exit test mode.\n");

  unsigned long lastDisplayUpdate = 0;
  bool leftState, rightState;
  bool leftPressed = false, rightPressed = false;

  while (true) {
    // Check for exit command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'q' || c == 'Q') {
        Serial.println("\nExiting test mode...");
        break;
      }
    }

    // Read current states (HIGH = not pressed, LOW = pressed)
    leftState = digitalRead(LIMIT_SWITCH_LEFT);
    rightState = digitalRead(LIMIT_SWITCH_RIGHT);

    // Track if switches have been pressed
    if (!leftState)
      leftPressed = true;
    if (!rightState)
      rightPressed = true;

    // Update display every 100ms
    if (millis() - lastDisplayUpdate >= 100) {
      lastDisplayUpdate = millis();

      // Serial output
      Serial.print("Left: ");
      Serial.print(leftState ? "HIGH (OK)     " : "LOW (PRESSED)");
      Serial.print("  |  Right: ");
      Serial.println(rightState ? "HIGH (OK)     " : "LOW (PRESSED)");

      // OLED display
      if (oledFound) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);

        display.println(F("LIMIT SWITCH TEST"));
        display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

        display.setCursor(0, 15);
        display.setTextSize(2);

        // Left switch
        display.print(F("L: "));
        if (leftState) {
          display.println(F("OK"));
        } else {
          display.println(F("PRESS"));
        }

        // Right switch
        display.print(F("R: "));
        if (rightState) {
          display.println(F("OK"));
        } else {
          display.println(F("PRESS"));
        }

        // Status
        display.setTextSize(1);
        display.setCursor(0, 50);
        display.print(F("Tested: L:"));
        display.print(leftPressed ? "Y" : "N");
        display.print(F(" R:"));
        display.println(rightPressed ? "Y" : "N");

        display.display();
      }
    }

    delay(10);
  }

  // Summary
  Serial.println("\n========================================");
  Serial.println("TEST SUMMARY:");
  Serial.print("Left switch tested: ");
  Serial.println(leftPressed ? "YES" : "NO");
  Serial.print("Right switch tested: ");
  Serial.println(rightPressed ? "YES" : "NO");

  if (leftPressed && rightPressed) {
    Serial.println("\n✓ Both switches working correctly!");
  } else {
    Serial.println("\n⚠ Warning: Not all switches were tested.");
  }
  Serial.println("========================================\n");
}

// ========================================
// Display Functions
// ========================================

void displayError(const char *error) {
  if (!oledFound)
    return;

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("ERROR!"));
  display.setTextSize(1);
  display.println(error);
  display.display();
}

void displayCalibrationStatus(const char *message) {
  if (!oledFound)
    return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(F("CALIBRATION"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 15);
  display.setTextSize(2);
  display.println(message);

  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print(F("L-Limit: "));
  display.println(steeringController.getLeftLimitState() ? "PRESS" : "OK");

  display.print(F("R-Limit: "));
  display.println(steeringController.getRightLimitState() ? "PRESS" : "OK");

  display.print(F("Encoder: "));
  display.println(steeringController.getEncoderCount());

  display.display();
}

void displayCommandReceived() {
  if (!oledFound)
    return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(F("CMD RECEIVED!"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 15);
  display.print(F("Linear:  "));
  display.print(lastLinearVel, 2);
  display.println(F(" m/s"));

  display.print(F("Angular: "));
  display.print(lastAngularVel, 2);
  display.println(F(" rad/s"));

  unsigned long timeSince = millis() - lastCmdVelTime;
  display.print(F("Age: "));
  display.print(timeSince);
  display.println(F("ms"));

  // Show steering angle with encoder feedback
  display.print(F("Target: "));
  display.print(steeringController.getTargetAngle(), 1);
  display.println(F(" deg"));

  display.print(F("Current: "));
  display.print(steeringController.getCurrentAngle(), 1);
  display.println(F(" deg"));

  display.display();
}

void displayLimitSwitchStatus() {
  if (!oledFound)
    return;

  // This is called from updateDisplay, so we just add to existing display
  display.print(F("L: "));
  display.print(steeringController.getLeftLimitState() ? "HIT" : "OK ");
  display.print(F(" R: "));
  display.println(steeringController.getRightLimitState() ? "HIT" : "OK ");
}

void displayConnectivityStatus() {
  if (!oledFound)
    return;

  // This is called from updateDisplay
  unsigned long timeSince = millis() - lastCmdVelTime;

  if (cmdVelReceived && timeSince < 2000) {
    display.println(F("ROS2: CONNECTED"));
  } else if (cmdVelReceived) {
    display.print(F("ROS2: IDLE ("));
    display.print(timeSince / 1000);
    display.println(F("s)"));
  } else {
    display.println(F("ROS2: WAITING..."));
  }
}

void updateDisplay() {
  if (!oledFound)
    return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (emergencyStopActive) {
    displayError("EMERGENCY STOP");
    return;
  }

  // Header
  display.println(F("UGV STATUS"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 15);

  // Calibration state
  display.print(F("Cal: "));
  display.println(steeringController.getCalibrationStateString());

  // Steering info with encoder feedback
  display.print(F("Ang: "));
  display.print(steeringController.getCurrentAngle(), 1);
  display.print(F("/"));
  display.println(steeringController.getTargetAngle(), 1);

  // Encoder count
  display.print(F("Enc: "));
  display.println(steeringController.getEncoderCount());

  // Velocity
  display.print(F("Vel: "));
  display.println(drivingController.getCurrentVelocity(), 2);

  // Limit switches
  displayLimitSwitchStatus();

  // Connectivity
  displayConnectivityStatus();

  display.display();
}

// ========================================
// Arduino Setup and Loop
// ========================================

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    oledFound = false;
  } else {
    Serial.println(F("OLED Found"));
    oledFound = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Booting UGV..."));
    display.display();
  }

  delay(1000);

  Serial.println("UGV ESP32 Controller Starting...");
  Serial.println("\n========================================");
  Serial.println("STARTUP LIMIT SWITCH CHECK");
  Serial.println("========================================");

  // Check limit switch states at startup
  // Note: Switches are connected to GND, so HIGH = not pressed, LOW = pressed
  bool leftState = digitalRead(LIMIT_SWITCH_LEFT);
  bool rightState = digitalRead(LIMIT_SWITCH_RIGHT);

  Serial.print("Left Limit Switch (Pin 32, GND-connected): ");
  Serial.println(leftState ? "HIGH (Not Pressed)" : "LOW (PRESSED!)");
  Serial.print("Right Limit Switch (Pin 33, GND-connected): ");
  Serial.println(rightState ? "HIGH (Not Pressed)" : "LOW (PRESSED!)");

  if (!leftState || !rightState) {
    Serial.println("\n⚠ WARNING: Limit switch(es) pressed at startup!");
    Serial.println("Please check physical alignment.\n");
  } else {
    Serial.println("✓ Limit switches OK (GND-connected)\n");
  }
  Serial.println("========================================\n");

  // Option to enter test mode - MUST complete within 3 seconds
  Serial.println(
      "Send 't' within 3 seconds to enter limit switch test mode...");
  Serial.println(
      "Send 's' within 3 seconds to enter steering angle test mode...");
  unsigned long startWait = millis();
  bool testModeRequested = false;
  while (millis() - startWait < 3000) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 't' || c == 'T') {
        testModeRequested = true;
        break;
      }
      if (c == 's' || c == 'S') {
        testModeRequested = true;
        break;
      }
    }
    delay(10);
  }

  // Clear any remaining serial data
  while (Serial.available() > 0) {
    Serial.read();
  }

  // Skip automatic limit test if user requested manual test mode
  if (!testModeRequested) {
    Serial.println("Performing automatic limit switch test...");
    autoTestLimitSwitches();
  }

  // Initialize controllers (AFTER any test mode, BEFORE calibration)
  Serial.println("Initializing controllers...");
  steeringController.begin();
  drivingController.begin();
  SteeringController::instance_ = &steeringController;

  // Set startup grace period flag (must be set before calibration)
  systemInitializeTime = millis();
  inStartupGracePeriod = true;
  steeringController.setStartupGracePeriod(true);
  Serial.println("⏱ Startup grace period enabled");

  // Check if already calibrated
  if (steeringController.getCalibrationState() !=
      SteeringController::CALIBRATED) {
    Serial.println("No calibration found. Starting calibration...");
    performCalibration();
  } else {
    Serial.println("Calibration loaded from memory.");
    systemInitialized = true;
  }

  // Clear OLED boot message and display ready state
  if (oledFound) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Initializing"));
    display.println(F("micro-ROS..."));
    display.setCursor(0, 30);
    display.println(F("Please wait..."));
    display.display();
  }

  // Clear limit switch flags before micro-ROS setup
  // After calibration, limit switches may have been pressed
  // This prevents false emergency stop triggers on micro-ROS connection
  Serial.println("Clearing limit switch flags...");
  steeringController.clearLimitSwitchFlags();
  delay(100);

  // Setup micro-ROS
  Serial.println("Initializing micro-ROS...");
  int microRosRetries = 0;
  bool rosInitSuccess = setupMicroROS();
  while (!rosInitSuccess && microRosRetries < 5) {
    Serial.println("Failed to initialize micro-ROS. Retrying...");
    microRosRetries++;
    delay(1000);
    rosInitSuccess = setupMicroROS();
  }

  if (!rosInitSuccess) {
    Serial.println("⚠ Failed to initialize micro-ROS after 5 attempts!");
    if (oledFound) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("micro-ROS FAILED"));
      display.setCursor(0, 20);
      display.println(F("Check connection"));
      display.display();
    }
    // Continue anyway - system can work without ROS2
    systemInitialized = true;
  } else {
    Serial.println("✓ micro-ROS initialized successfully!");
    systemInitialized = true;
  }

  // Update display after micro-ROS connection
  if (oledFound) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("READY!"));
    display.setTextSize(1);
    display.setCursor(0, 25);
    display.println(F("Waiting for"));
    display.println(F("ROS2 commands..."));
    display.display();
    delay(2000);
  }

  Serial.println("✓ System initialization complete!");
  Serial.println("Ready to accept ROS2 commands.\n");
}

void loop() {
  // Feed the watchdog timer - prevents ESP32 reset
  esp_task_wdt_reset();
  
  // Spin executor to handle callbacks (with timeout to prevent hang)
  if (systemInitialized) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }

  // Control loop
  unsigned long currentTime = millis();

  if (currentTime - lastControlUpdate >= CONTROL_PERIOD_MS) {
    lastControlUpdate = currentTime;

    checkErrorStates();

    if (systemInitialized && !emergencyStopActive) {
      updateControllers();
    }
  }

  // Status publishing
  if (currentTime - lastStatusPublish >= STATUS_PERIOD_MS) {
    lastStatusPublish = currentTime;
    publishStatus();
  }

  // Display update (separate from status publishing for better control)
  if (currentTime - lastDisplayUpdate >= DISPLAY_PERIOD_MS) {
    lastDisplayUpdate = currentTime;

    // Show command received popup for 1 second after receiving a command
    // Only after system is fully initialized and calibrated
    if (systemInitialized && cmdVelReceived && (currentTime - lastCmdVelTime) < 1000) {
      displayCommandReceived();
    } else {
      updateDisplay();
    }
  }
}