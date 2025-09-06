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

<<<<<<< HEAD
## Version 10.3 (20250625-090416)

### Breaking Changes
* The behavior of setGlobalErrorMsg() is changed.  Note that this is an SDK internal method that is not 
  meant to be used by team software or third party libraries.  Teams or libraries using this method should
  find another means to communicate failure.  The design intent of setGlobalErrorMsg() is to report an 
  error and force the user to restart the robot, which in certain circumstances when used inappropriately
  could cause a robot to continue running while Driver Station controls are disabled.  To prevent this,
  processing of a call to setGlobalErrorMsg() is deferred until the robot is in a known safe state.  This may
  mean that a call to setGlobalErrorMsg() that does not also result in stopping a running OpMode will appear
  as though nothing happened until the robot is stopped, at which point, if clearGlobalErrorMsg() has not 
  been called the message will appear on the Driver Station and a restart will be required.
  Addresses issue [1381](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1381)
* Fixes getLatestResult in Limelight3A so if the Limelight hasn't provided data yet, it still returns an LLResult but valid will be false
  * If you previously used to check and see if this was `null` to see if the Limelight had been contacted, you now need to use `isValid()` on the result.  That is because now it always returns an LLResult even before it talks to the Limelight, but if it doesn't have valid data, the `isValid()` will be `false`.
* Changed all omni samples to use front_left_drive, front_right_drive, back_left_drive, back_right_drive
  * This is only breaking for you if you copy one of the changed samples to your own project and expect to use the same robot configuration as before.

### Known Issues
* The redesigned OnBotJava new file workflow allows the user to use a lowercase letter as the first character of a filename.
  This is a regression from 10.2 which required the first character to be uppercase.  Software will build, but if the user tries
  to rename the file, the rename will fail.

### Enhancements
* Improved the OBJ new file creation flow workflow. The new flow allows you to easily use samples, craft new custom OpModes and make new Java classes.
* Added support for gamepad edge detection.
  * A new sample program `ConceptGamepadEdgeDetection` demonstrates its use.
* Adds a blackboard member to the Opmode that maintains state between opmodes (but not between robot resets).  See the ConceptBlackboard sample for how to use it.
* Updated PredominantColorProcessor to also return the predominant color in RGB, HSV and YCrCb color spaces.  Updated ConceptVisionColorSensor sample OpMode to display the getAnalysis() result in all three color spaces.
* Adds support for the GoBilda Pinpoint 
  * Also adds `SensorGoBildaPinpoint` sample to show how to use it
* Added `getArcLength()` and `getCircularity()` to ColorBlobLocatorProcessor.Blob.  Added BY_ARC_LENGTH and BY_CIRCULARITY as additional BlobCriteria.
* Added `filterByCriteria()` and `sortByCriteria()` to ColorBlobLocatorProcessor.Util.
  * The filter and sort methods for specific criteria have been deprecated.
  * The updated sample program `ConceptVisionColorLocator` provides more details on the new syntax.
* Add Help menu item and Help page that is available when connected to the robot controller via Program and Manage. The Help page has links to team resources such as [FTC Documentation](https://ftc-docs.firstinspires.org/), [FTC Discussion Forums](https://ftc-community.firstinspires.org), [Java FTC SDK API Documentation](https://javadoc.io/doc/org.firstinspires.ftc), and [FTC Game Information](https://ftc.game/).
* Self inspection changes:
  * List both the Driver Station Name and Robot Controller Name when inspecting the Driver Station.
  * Report if the team number portion of the device names do not match.
  * -rc is no longer valid as part of a Robot Controller name, must be -RC.
  * Use Robot Controller Name or Driver Station Name labels on the inspection screens instead of WIFI Access Point or WIFI Direct Name.

### Bug Fixes
* Fixes issue [1478](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1478) in AnnotatedHooksClassFilter that ignored exceptions if they occur in one of the SDK app hooks.
* Fix initialize in distance sensor (Rev 2m) to prevent bad data in first call to getDistance.
* Fixes issue [1470](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1470) Scaling a servo range is now irrespective of reverse() being called.  For example, if you set the scale range to [0.0, 0.5] and the servo is reversed, it will be from 0.5 to 0.0, NOT 1.0 to 0.5.
* Fixes issue [1232](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1232), a rare race condition where using the log rapidly along with other telemetry could cause a crash.

## Version 10.2 (20250121-174034)
=======
---
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97

## 🙌 Acknowledgments

- FTC SDK by FIRST & REV Robotics  
- Pedro Pathing by FTC team Scott's Bots #10158 (https://ftc-events.firstinspires.org/team/10158)
- Inspiration from the global FTC community
- FTCRobotController from https://github.com/FIRST-Tech-Challenge/FtcRobotController

---
### Made by FTC team Cyber Salam #26903. See more at https://cybersalam.odoo.com
