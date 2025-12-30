// ============================================================================
// UGV Firmware Configuration
// ============================================================================
// Consolidated configuration for ESP32-based Ackermann UGV
// ============================================================================

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// ROBOT PHYSICAL PARAMETERS
// ============================================================================

#define WHEELBASE_M 0.6f      // Front-to-rear axle distance (meters)
#define TRACK_WIDTH_M 1.16f   // Left-to-right wheel distance (meters)
#define MAX_VELOCITY_MPS 1.0f // Maximum linear velocity (m/s)
#define MAX_ACCELERATION 0.5f // Max acceleration (m/s per update at 100Hz)

// ============================================================================
// STEERING CONFIGURATION
// ============================================================================

// Steering angle limits
#define MAX_STEERING_ANGLE_DEG 10.0f  // Maximum steering angle (degrees)
#define MIN_STEERING_ANGLE_DEG -10.0f // Minimum steering angle (degrees)
#define SOFT_LIMIT_ANGLE_DEG 8.0f     // Soft limit buffer (degrees)

// Encoder configuration - values from calibration (~12700 pulses for 20
// degrees)
#define ENCODER_PPR 600            // Pulses per revolution (encoder spec)
#define PULSES_PER_10_DEGREES 6350 // Calibrated: 12700/2 = 6350
#define PULSES_PER_DEGREE 635.0f   // Calibrated: 12700/20 = 635
#define SOFT_LIMIT_PULSES 5080     // ±8 degrees = ±5080 pulses (635 * 8)

// PID tuning parameters
#define STEERING_KP 2.5f
#define STEERING_KI 0.1f
#define STEERING_KD 0.5f
#define STEERING_PID_SAMPLE_MS 10 // PID sample time (ms)

// Motor direction inversion (set to -1 if motor runs backwards)
#define STEERING_MOTOR_INVERT -1

// Calibration
#define CALIBRATION_SPEED 100        // PWM value during calibration
#define CALIBRATION_TIMEOUT_MS 10000 // Calibration timeout (10 sec)

// ============================================================================
// PIN ASSIGNMENTS - ESP32 DOIT DevKit V1
// ============================================================================

// Steering Motor (BTS7960)
#define PIN_STEERING_LPWM 25 // Left PWM (forward)
#define PIN_STEERING_RPWM 26 // Right PWM (reverse)

// Driving Motor (BTS7960)
#define PIN_DRIVING_LPWM 27 // Left PWM (forward)
#define PIN_DRIVING_RPWM 14 // Right PWM (reverse)

// Steering Encoder (input-only pins)
#define PIN_ENCODER_A 34 // Channel A
#define PIN_ENCODER_B 35 // Channel B

// Limit Switches (wired to GND, use INPUT_PULLUP)
#define PIN_LIMIT_LEFT 32  // Left limit switch
#define PIN_LIMIT_RIGHT 33 // Right limit switch

// I2C Bus (OLED + IMU)
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// ============================================================================
// PWM CONFIGURATION
// ============================================================================

#define PWM_FREQUENCY 1000 // 1 kHz
#define PWM_RESOLUTION 8   // 8-bit (0-255)
#define PWM_MAX_VALUE 255
#define PWM_MIN_VALUE 0
#define MOTOR_DEADBAND 20 // Minimum PWM to move motor

// PWM Channels
#define PWM_CH_STEERING_L 0
#define PWM_CH_STEERING_R 1
#define PWM_CH_DRIVING_L 2
#define PWM_CH_DRIVING_R 3

// ============================================================================
// DISPLAY CONFIGURATION (SSD1306 OLED)
// ============================================================================

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_I2C_ADDRESS 0x3C
#define OLED_RESET_PIN -1 // No reset pin

// ============================================================================
// IMU CONFIGURATION (MPU6050 / GY-87)
// ============================================================================

#define IMU_ACCEL_RANGE MPU6050_RANGE_4_G
#define IMU_GYRO_RANGE MPU6050_RANGE_500_DEG
#define IMU_FILTER_BW MPU6050_BAND_21_HZ
#define IMU_COMPLEMENTARY_ALPHA 0.98f // Filter coefficient

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

#define LOOP_PERIOD_MS 10           // Main loop period (100 Hz)
#define STATUS_PERIOD_MS 200        // Status publishing (5 Hz) - reduced from 10Hz
#define HEARTBEAT_PERIOD_MS 200     // Heartbeat publishing (5 Hz) - reduced from 10Hz
#define STEERING_ANGLE_PERIOD_MS 50 // Steering angle publishing (20 Hz)
#define IMU_PERIOD_MS 20            // IMU publishing (50 Hz)
#define DISPLAY_PERIOD_MS 200       // Display update (5 Hz)

// ============================================================================
// SAFETY CONFIGURATION
// ============================================================================

#define CMD_TIMEOUT_MS 500           // Command timeout (ms)
#define STARTUP_GRACE_PERIOD_MS 2000 // Grace period after init (ms)
#define LIMIT_DEBOUNCE_MS 50         // Limit switch debounce (ms)

// ============================================================================
// micro-ROS CONFIGURATION
// ============================================================================

#define MICROROS_SERIAL_BAUD 115200
#define NODE_NAME "ugv_esp32_node"
#define TOPIC_CMD_VEL "/cmd_vel"
#define TOPIC_STATUS "/ugv/status"
#define TOPIC_HEARTBEAT "/ugv/heartbeat"
#define TOPIC_STEERING_ANGLE "/ugv/steering_angle"
#define TOPIC_IMU "/ugv/imu"

#endif // CONFIG_H
