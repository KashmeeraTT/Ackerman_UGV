# UGV ESP32 Firmware

ESP32 firmware for Ackermann UGV motor control with micro-ROS integration.

## Features

- **Steering Control**: Encoder-based PID control with limit switch protection
- **Drive Motor Control**: PWM-based velocity control via BTS7960 drivers
- **micro-ROS**: ROS2 integration over USB serial
- **OLED Display**: Real-time status with mode, connection, and sensor info
- **Fail-Safe**: Command timeout, E-stop, and startup centering

## Hardware

| Component | Specification |
|-----------|--------------|
| MCU | ESP32 DOIT DevKit V1 |
| Motor Driver | BTS7960 (x2) |
| Steering Encoder | Rotary encoder (dynamic calibration) |\n| Wheel Encoders | 2x quadrature encoders for odometry |\n| IMU | GY-87 (MPU6050) |\n| OLED | SSD1306 128x64 I2C |\n| Limit Switches | 2x normally open |

## ROS2 Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/cmd_vel` | Twist | Subscriber | Velocity commands |
| `/ugv/status` | String | 5 Hz | Status information |
| `/ugv/heartbeat` | Bool | 5 Hz | Connection monitor |
| `/ugv/steering_angle` | Float32 | 20 Hz | Current steering angle |
| `/ugv/imu` | Imu | 50 Hz | IMU sensor data |

## Robot Dimensions

```
Wheelbase:       0.6 m
Track Width:     1.16 m
Max Steering:    ±10° (hardware), ±8° (soft limit)
Min Turn Radius: ~4.27 m (at 8° soft limit)
```

## Building

```bash
# Install PlatformIO
pip install platformio

# Build
pio run

# Upload
pio run --target upload
```

## Pin Configuration

See `include/config.h` and `include/pin_config.h` for configuration.

## Safety Features

1. **Command Timeout**: Motors stop if no `/cmd_vel` for 500ms
2. **E-Stop**: Auto-stop when limit switch triggered
3. **Steering Clamp**: Hardware limits at ±10°, software at ±8°
4. **Startup Calibration**: Auto-calibrates encoder and centers steering

## File Structure

```
firmware/
├── src/
│   ├── main.cpp                  # Entry point and main loop
│   ├── controllers/
│   │   ├── steering_controller.cpp
│   │   └── drive_controller.cpp
│   ├── drivers/
│   │   ├── bts7960.cpp           # Motor driver
│   │   ├── display.cpp           # OLED display
│   │   └── imu.cpp               # IMU sensor
│   ├── hal/
│   │   ├── encoder.cpp
│   │   └── limit_switch.cpp
│   ├── comms/
│   │   └── ros_bridge.cpp        # micro-ROS bridge
│   └── safety/
│       └── safety_monitor.cpp
├── include/
│   ├── config.h                  # Main configuration
│   ├── pin_config.h              # GPIO pin definitions
│   ├── controllers/
│   │   ├── steering_controller.h
│   │   └── drive_controller.h
│   ├── drivers/
│   │   ├── bts7960.h
│   │   ├── display.h
│   │   └── imu.h
│   ├── hal/
│   │   ├── encoder.h
│   │   └── limit_switch.h
│   ├── comms/
│   │   └── ros_bridge.h
│   └── safety/
│       └── safety_monitor.h
├── lib/                          # External libraries
├── platformio.ini                # PlatformIO config
└── README.md                     # This file
```

## License

MIT License
