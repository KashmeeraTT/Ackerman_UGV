# References Folder

Complete documentation for the Ackerman UGV steering control system.

## 📚 Quick Navigation

### Quick Start & Overview
- **[COMPLETE_SUMMARY.md](./COMPLETE_SUMMARY.md)** - Full project overview with quick start guide
- **[KEYBOARD_QUICK_REFERENCE.md](./KEYBOARD_QUICK_REFERENCE.md)** - One-page keyboard command reference
- **[STEERING_QUICK_REFERENCE.md](./STEERING_QUICK_REFERENCE.md)** - Quick encoder specs and test mode guide

### Complete Guides
- **[KEYBOARD_TELEOP_GUIDE.md](./KEYBOARD_TELEOP_GUIDE.md)** - Complete keyboard teleoperation documentation
- **[STEERING_IMPLEMENTATION.md](./STEERING_IMPLEMENTATION.md)** - Complete steering control implementation details
- **[KEYBOARD_VISUAL_GUIDE.md](./KEYBOARD_VISUAL_GUIDE.md)** - Flowcharts, diagrams, and keyboard layouts

### Advanced Topics
- **[INTEGRATION_GUIDE.md](./INTEGRATION_GUIDE.md)** - System architecture and data flow analysis
- **[CHANGELOG.md](./CHANGELOG.md)** - Detailed list of all code modifications

## 🎯 Choose Your Path

**Just want to use it?**
→ Read: KEYBOARD_QUICK_REFERENCE.md, then COMPLETE_SUMMARY.md

**Want to understand the system?**
→ Read: KEYBOARD_TELEOP_GUIDE.md, STEERING_IMPLEMENTATION.md, KEYBOARD_VISUAL_GUIDE.md

**Need technical deep dive?**
→ Read: INTEGRATION_GUIDE.md, CHANGELOG.md, then review source code

## 📋 File Descriptions

| File | Purpose | Length |
|------|---------|--------|
| COMPLETE_SUMMARY.md | Project overview and quick start | 400+ lines |
| KEYBOARD_QUICK_REFERENCE.md | One-page keyboard commands | 1 page |
| STEERING_QUICK_REFERENCE.md | Quick encoder specs and test mode | 1 page |
| KEYBOARD_TELEOP_GUIDE.md | Complete keyboard guide with examples | 500+ lines |
| STEERING_IMPLEMENTATION.md | Technical steering details | 400+ lines |
| KEYBOARD_VISUAL_GUIDE.md | Flowcharts and visual diagrams | 600+ lines |
| INTEGRATION_GUIDE.md | System architecture and data flow | 800+ lines |
| CHANGELOG.md | Detailed code changes | 600+ lines |

## 🚀 Quick Start Commands

```bash
# Upload firmware
platformio run -e esp32doit-devkit-v1 -t upload

# Start keyboard teleop
ros2 run ugv_teleop teleop_keyboard

# Test steering (during ESP32 startup, send 's' within 3 seconds)
Commands: a, d, s, h, y, q
```

## ⌨️ Essential Keyboard Commands

**Steering Angle Control (NEW):**
- `a` = Decrease by 2°
- `d` = Increase by 2°
- `s` = Center (0°)
- `h` = Full left (-10°)
- `y` = Full right (+10°)

**Movement:**
- `i` = Forward
- `j`/`l` = Turn left/right
- `k`/`space` = Stop

**Speed Adjustment:**
- `q`/`z` = ±10% all speeds
- `w`/`x` = ±10% linear only
- `e`/`c` = ±10% angular only

## 🔧 System Specifications

| Specification | Value |
|---------------|-------|
| Encoder Calibration | 500 pulses = 10° |
| Encoder Resolution | 50 pulses/degree |
| Steering Range | -10° to +10° |
| Control Loop | 100 Hz (10ms) |
| PID Gains | P=2.0, I=0.5, D=0.1 |
| Max Linear Speed | 2.0 m/s |
| Max Angular Velocity | 3.0 rad/s |

## 📊 Documentation Organization

```
references/
├─ README.md (this file)
│
├─ Quick References (1-page guides):
│  ├─ KEYBOARD_QUICK_REFERENCE.md
│  └─ STEERING_QUICK_REFERENCE.md
│
├─ Complete Guides (500+ line guides):
│  ├─ KEYBOARD_TELEOP_GUIDE.md
│  ├─ STEERING_IMPLEMENTATION.md
│  ├─ KEYBOARD_VISUAL_GUIDE.md
│  └─ COMPLETE_SUMMARY.md
│
└─ Technical Reference:
   ├─ INTEGRATION_GUIDE.md
   └─ CHANGELOG.md
```

## ✨ Key Features

✅ Direct steering angle control via keyboard (5 new keys)
✅ Encoder feedback with 50 pulses per degree resolution
✅ Real-time feedback on OLED and terminal
✅ Complete ROS2 integration
✅ Full backward compatibility with original functions
✅ Interactive test mode for verification
✅ Production-ready code quality
✅ Comprehensive documentation

## 🔍 Finding What You Need

**"I need to get started quickly"**
→ `COMPLETE_SUMMARY.md` → "Quick Start Guide"

**"What keyboard keys can I use?"**
→ `KEYBOARD_QUICK_REFERENCE.md`

**"How do I test the encoder?"**
→ `STEERING_QUICK_REFERENCE.md`

**"I want to understand how steering works"**
→ `STEERING_IMPLEMENTATION.md`

**"How do keyboard inputs reach the motor?"**
→ `INTEGRATION_GUIDE.md` → "Data Flow" section

**"What exactly changed in the code?"**
→ `CHANGELOG.md`

**"I want to see flowcharts and diagrams"**
→ `KEYBOARD_VISUAL_GUIDE.md`

**"I need the complete keyboard guide"**
→ `KEYBOARD_TELEOP_GUIDE.md`

## 💡 Tips

- **New to the project?** Start with `COMPLETE_SUMMARY.md`
- **Quick lookup needed?** Use the quick reference files (1 page each)
- **Building something?** Check `INTEGRATION_GUIDE.md` for architecture
- **Debugging an issue?** Look in `CHANGELOG.md` for what changed
- **Need diagrams?** See `KEYBOARD_VISUAL_GUIDE.md`

## 📞 Support

If you can't find what you're looking for:
1. Check the table of contents in the relevant file
2. Search for keywords using Ctrl+F
3. Refer to the quick navigation links above
4. Check `CHANGELOG.md` for code modifications
5. Review source code comments in main.cpp, SteeringController.h, teleop_keyboard.py

---

**Last Updated**: December 2, 2025
**Version**: 1.0 - Complete
**Status**: Production Ready ✅
