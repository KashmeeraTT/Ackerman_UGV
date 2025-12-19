// ============================================================================
// OLED Display Driver Implementation
// ============================================================================

#include "drivers/display.h"
#include <Wire.h>

Display::Display()
    : oled_(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN), available_(false) {
}

bool Display::begin() {
  if (!oled_.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println("SSD1306 not found");
    return false;
  }

  available_ = true;
  oled_.clearDisplay();
  oled_.setTextColor(SSD1306_WHITE);
  oled_.display();
  Serial.println("OLED initialized");
  return true;
}

void Display::clear() {
  if (!available_)
    return;
  oled_.clearDisplay();
}

void Display::show() {
  if (!available_)
    return;
  oled_.display();
}

void Display::showBoot(const char *message) {
  if (!available_)
    return;

  clear();
  oled_.setTextSize(1);
  oled_.setCursor(0, 0);
  oled_.println(F("UGV Firmware"));
  oled_.println();
  oled_.println(message);
  show();
}

void Display::showCalibration(const char *message, bool leftLimit,
                              bool rightLimit, long encoderCount) {
  if (!available_)
    return;

  clear();
  oled_.setTextSize(1);
  oled_.setCursor(0, 0);
  oled_.println(F("CALIBRATION"));
  oled_.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  oled_.setCursor(0, 15);
  oled_.setTextSize(2);
  oled_.println(message);

  oled_.setTextSize(1);
  oled_.setCursor(0, 35);
  oled_.print(F("L: "));
  oled_.println(leftLimit ? "PRESS" : "OK");
  oled_.print(F("R: "));
  oled_.println(rightLimit ? "PRESS" : "OK");
  oled_.print(F("Enc: "));
  oled_.println(encoderCount);

  show();
}

void Display::showStatus(const char *mode, bool rosConnected,
                         unsigned long cmdAgeMs, float currentAngle,
                         float targetAngle, float velocity, long encoderCount,
                         bool leftLimit, bool rightLimit,
                         const char *statusCode) {
  if (!available_)
    return;

  clear();

  // Row 1: Mode (large text)
  oled_.setTextSize(2);
  oled_.setCursor(0, 0);
  oled_.print(mode);

  // Row 2: Connection + Command Age
  oled_.setTextSize(1);
  oled_.setCursor(0, 20);
  oled_.print(F("ROS:"));
  oled_.print(rosConnected ? "OK " : "NO ");
  oled_.print(F("Cmd:"));
  if (cmdAgeMs < 1000) {
    oled_.print(cmdAgeMs);
    oled_.print(F("ms"));
  } else {
    oled_.print(cmdAgeMs / 1000);
    oled_.print(F("s"));
  }

  // Row 3: Steering angle
  oled_.setCursor(0, 32);
  oled_.print(F("Steer:"));
  oled_.print(currentAngle, 1);
  oled_.print(F("/"));
  oled_.print(targetAngle, 1);

  // Row 4: Speed and encoder
  oled_.setCursor(0, 42);
  oled_.print(F("Speed:"));
  oled_.print(velocity, 2);
  oled_.print(F("m/s"));
  oled_.setCursor(80, 42);
  oled_.print(F("E:"));
  oled_.print(encoderCount);

  // Row 5: Limits and status
  oled_.setCursor(0, 54);
  oled_.print(F("L:"));
  oled_.print(leftLimit ? "HIT" : "OK ");
  oled_.print(F(" R:"));
  oled_.print(rightLimit ? "HIT" : "OK ");
  oled_.setCursor(90, 54);
  oled_.print(statusCode);

  show();
}

void Display::showError(const char *error) {
  if (!available_)
    return;

  clear();
  oled_.setTextSize(2);
  oled_.setCursor(0, 0);
  oled_.println(F("ERROR!"));
  oled_.setTextSize(1);
  oled_.println(error);
  show();
}

void Display::showCalibrationProgress(int step, const char *stepName,
                                      int progress, int timeoutSec,
                                      long encoderCount) {
  if (!available_)
    return;

  clear();

  // Header with step indicator
  oled_.setTextSize(1);
  oled_.setCursor(0, 0);
  oled_.print(F("CALIBRATION ["));
  oled_.print(step);
  oled_.print(F("/4]"));
  oled_.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // Step name in large text
  oled_.setTextSize(2);
  oled_.setCursor(0, 14);
  oled_.println(stepName);

  // Progress bar (100px wide, 8px tall)
  oled_.setTextSize(1);
  oled_.setCursor(0, 34);
  oled_.print(F("Progress:"));
  int barWidth = map(progress, 0, 100, 0, 100);
  oled_.drawRect(0, 44, 102, 10, SSD1306_WHITE);
  oled_.fillRect(1, 45, barWidth, 8, SSD1306_WHITE);

  // Right side: percentage
  oled_.setCursor(106, 46);
  oled_.print(progress);
  oled_.print(F("%"));

  // Bottom: Timeout and encoder
  oled_.setCursor(0, 56);
  oled_.print(F("Timeout:"));
  oled_.print(timeoutSec);
  oled_.print(F("s  Enc:"));
  oled_.print(encoderCount);

  show();
}

void Display::showCalibrationSuccess(long totalPulses, float pulsesPerDeg) {
  if (!available_)
    return;

  clear();

  // Success header
  oled_.setTextSize(2);
  oled_.setCursor(10, 0);
  oled_.println(F("SUCCESS!"));
  oled_.drawLine(0, 18, 128, 18, SSD1306_WHITE);

  // Results
  oled_.setTextSize(1);
  oled_.setCursor(0, 24);
  oled_.print(F("Total range: "));
  oled_.print(totalPulses);
  oled_.println(F(" pulses"));

  oled_.setCursor(0, 36);
  oled_.print(F("Pulses/deg: "));
  oled_.println(pulsesPerDeg, 1);

  oled_.setCursor(0, 48);
  oled_.println(F("Centering wheel..."));

  show();
}

void Display::showCalibrationFailed(const char *reason, int step) {
  if (!available_)
    return;

  clear();

  // Failure header
  oled_.setTextSize(2);
  oled_.setCursor(20, 0);
  oled_.println(F("FAILED!"));
  oled_.drawLine(0, 18, 128, 18, SSD1306_WHITE);

  // Error info
  oled_.setTextSize(1);
  oled_.setCursor(0, 24);
  oled_.print(F("Step "));
  oled_.print(step);
  oled_.println(F(" error:"));

  oled_.setCursor(0, 36);
  oled_.println(reason);

  oled_.setCursor(0, 52);
  oled_.println(F("Check wiring/switches"));

  show();
}
