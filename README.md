# 🤖 FTC Robot Code: TeleOp & Autonomous

Welcome to our FTC robot code repository! This project contains the control software for our competition robot, including both TeleOp (manual control) and Autonomous (pre-programmed) modes. Built using the FTC SDK and written in Java, this codebase is designed for reliability, modularity, and performance on the field.

---

---

## 🎮 TeleOp Mode

**File:** `Main.java`  
This mode allows drivers to control the robot using gamepads during the match.

### Features:
- Tank or arcade drive (configurable)
- Arm and claw control
- Speed scaling for precision
- Sensor feedback (e.g., IMU, distance sensors)

---

## 🚀 Autonomous Mode

**Files:** `Auto.java`, etc.  
These routines run during the autonomous period of the match.

### Features:
- Path planning using Pedro Pathing
- Object detection (AprilTags, color sensors)
- Precise movement using encoders and IMU
- Modular routines for different starting positions

---

## 🛠️ Hardware Configuration

### Includes:
- Motors (drive, arm, intake)
- Servos (claw, wrist)
- Sensors (IMU, distance, color)
- Easy access methods for movement and control

---

## 📦 Dependencies

- FTC SDK (latest version)
- [Pedro Pathing](https://github.com/Pedro-Pathing/PedroPathing) for autonomous path planning
- OpenCV (optional for vision processing)

---

## 🧪 Testing & Deployment

1. Connect your Control Hub or Expansion Hub via USB.
2. Build and deploy using Android Studio.
3. Use the Driver Station app to select and run OpModes.

---

## 🧠 Contributing

We welcome contributions from team members and collaborators! Please follow our coding style and thoroughly test your code before submitting pull requests.

---

## 📜 License

This project is licensed under the MIT License. See `LICENSE.md` for details.

---

## 🙌 Acknowledgments

- FTC SDK by FIRST & REV Robotics  
- Pedro Pathing by FTC team Scott's Bots #10158 (https://ftc-events.firstinspires.org/team/10158)
- Inspiration from the global FTC community
- FTCRobotController from https://github.com/FIRST-Tech-Challenge/FtcRobotController

---
### Made by FTC team Cyber Salam #26903. See more at https://cybersalam.odoo.com
