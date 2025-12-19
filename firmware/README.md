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
| Steering Encoder | Rotary encoder with quadrature |
| OLED | SSD1306 128x64 I2C |
| Limit Switches | 2x normally open |

## ROS2 Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/cmd_vel` | Twist | Subscriber | Velocity commands |
| `/ugv/status` | String | 10 Hz | Status information |
| `/ugv/heartbeat` | Bool | 10 Hz | Connection monitor |
| `/ugv/steering_angle` | Float32 | 20 Hz | Current steering angle |

## Robot Dimensions

```
Wheelbase:       0.6 m
Track Width:     1.16 m
Max Steering:    ±10°
Min Turn Radius: ~3.40 m
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

See `include/pin_config.h` for pin assignments.

## Safety Features

1. **Command Timeout**: Motors stop if no `/cmd_vel` for 500ms
2. **E-Stop**: Auto-stop when limit switch triggered
3. **Steering Clamp**: Hardware limits at ±10°
4. **Startup Centering**: Steering auto-centers on boot

## File Structure

```
firmware/
├── src/
│   └── main.cpp          # Main firmware
├── include/
│   ├── pin_config.h      # Pin definitions
│   ├── MotorDriver.h     # BTS7960 driver
│   ├── SteeringController.h
│   └── DrivingController.h
├── lib/                  # Libraries
├── platformio.ini        # PlatformIO config
└── README.md            # This file
```

## License

MIT License
