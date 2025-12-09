#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ========================================
// ESP32 DOIT DevKit V1 Pin Configuration
// ========================================

// Steering Motor (BTS7960) - PWM capable pins
#define STEERING_LPWM_PIN 25 // Left PWM - Forward steering
#define STEERING_RPWM_PIN 26 // Right PWM - Reverse steering

// Driving Motor (BTS7960) - PWM capable pins
#define DRIVING_LPWM_PIN 27 // Left PWM - Forward drive
#define DRIVING_RPWM_PIN 14 // Right PWM - Reverse drive

// Steering Encoder - Input only pins (no pull-up/down needed)
#define ENCODER_A_PIN 34 // Encoder Channel A (input only)
#define ENCODER_B_PIN 35 // Encoder Channel B (input only)

// Driving Wheel Encoders
#define LEFT_ENCODER_A_PIN 36  // Input Only (Sensor VP)
#define LEFT_ENCODER_B_PIN 39  // Input Only (Sensor VN)
#define RIGHT_ENCODER_A_PIN 18 // General GPIO
#define RIGHT_ENCODER_B_PIN 19 // General GPIO

// Limit Switches - Interrupt capable pins with internal pull-up
#define LIMIT_SWITCH_LEFT 32  // Left limit switch (active LOW)
#define LIMIT_SWITCH_RIGHT 33 // Right limit switch (active LOW)

// PWM Configuration
#define PWM_FREQUENCY 1000 // 1 kHz PWM frequency
#define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

// PWM Channels (ESP32 has 16 channels)
#define STEERING_LPWM_CHANNEL 0
#define STEERING_RPWM_CHANNEL 1
#define DRIVING_LPWM_CHANNEL 2
#define DRIVING_RPWM_CHANNEL 3

// Motor Configuration
#define MAX_PWM_VALUE 255 // Maximum PWM value
#define MIN_PWM_VALUE 0   // Minimum PWM value
#define MOTOR_DEADBAND 20 // PWM deadband to prevent motor hum

// Encoder Configuration
#define ENCODER_PPR 600                    // Pulses per revolution
#define PULSES_PER_10_DEGREES 500          // 10 degrees = 500 pulses (as per requirement)
#define PULSES_PER_DEGREE (PULSES_PER_10_DEGREES / 10.0) // 50 pulses per degree

// Steering Safety Limits
#define MAX_STEERING_ANGLE_DEG 10.0  // Maximum steering angle (degrees)
#define MIN_STEERING_ANGLE_DEG -10.0 // Minimum steering angle (degrees)

// Calibration Configuration
#define CALIBRATION_SPEED 100        // PWM value during calibration
#define CALIBRATION_TIMEOUT_MS 10000 // Timeout for calibration (10 seconds)

#define CALIBRATION_TIMEOUT_MS 10000 // Timeout for calibration (10 seconds)

// I2C Configuration (OLED + GY87)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET_PIN -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS                                                         \
  0x3C // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#endif // PIN_CONFIG_H
