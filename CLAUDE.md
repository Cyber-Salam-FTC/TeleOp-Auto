# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FTC (FIRST Tech Challenge) robot control system for team Cyber Salam #26903. The codebase includes both TeleOp (manual control) and Autonomous modes, built on the FTC SDK using Java and Android Studio.

## Build Commands

**Build the project:**
```bash
./gradlew build
```

**Deploy to robot (requires connected Control Hub):**
```bash
./gradlew assembleDebug
```

The project uses Gradle with Android Studio. Main build files:
- [build.gradle](build.gradle) - Root build configuration
- [TeamCode/build.gradle](TeamCode/build.gradle) - Team-specific module
- [build.common.gradle](build.common.gradle) - Shared build definitions

## Code Architecture

### Module Structure

The project follows the standard FTC SDK structure with two main modules:

1. **FtcRobotController** - Base SDK module containing sample OpModes and framework code
2. **TeamCode** - Custom team code (primary development area)

All team-specific code is in [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/)

### OpMode Registration

OpModes are registered via annotations and must extend either `OpMode` (iterative) or `LinearOpMode`:
- `@TeleOp(name="Display Name")` - For driver-controlled modes
- `@Autonomous(name="Display Name")` - For autonomous routines
- `@Disabled` - Prevents OpMode from appearing in Driver Station

### Core OpModes

**TeleOp:** [MainOp.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MainOp.java)
- Main driver control mode
- Uses mecanum drive with trigger-based forward/backward control
- Gamepad 1: right trigger (forward), left trigger (backward), left stick X (strafe), right stick X (rotate)

**Autonomous:** [Auto.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto.java)
- Uses Pedro Pathing library for path following
- Defines paths as Bezier lines between poses
- Follower system handles path execution and localization

### Hardware Abstraction

**RobotHardware class:** [cybersalam/hardware/RobotHardware.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/cybersalam/hardware/RobotHardware.java)
- Centralizes hardware initialization and configuration
- Motor names: `leftFront`, `leftRear`, `rightFront`, `rightRear`
- Left motors are reversed, right motors are forward
- All motors use `RUN_USING_ENCODER` mode with `BRAKE` behavior

**MecanumDrive class:** [cybersalam/hardware/MecanumDrive.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/cybersalam/hardware/MecanumDrive.java)
- Implements mecanum drive kinematics
- `drive(forward, strafe, rotate)` method handles motor power calculations
- Includes automatic power scaling to maintain vector direction

### Pedro Pathing Integration

The autonomous system uses the Pedro Pathing library for advanced path following:

**Constants file:** [pedropathing/Constants.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedropathing/Constants.java)
- Contains all tuned parameters for the robot
- Defines `FollowerConstants` (PID coefficients, zero power acceleration)
- Defines `PathConstraints` (velocity, acceleration limits)
- Defines `PinpointConstants` (odometry pod configuration)
- Defines `MecanumConstants` (drivetrain configuration)
- Use `Constants.createFollower(hardwareMap)` to instantiate a configured Follower

**Tuning OpMode:** [pedropathing/Tuning.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedropathing/Tuning.java)
- Selection menu for various tuning routines
- Localization tuners (forward, lateral, turn)
- Velocity and acceleration tuners (automatic)
- PIDF tuners (manual)
- Test paths (line, triangle, circle)

### Path Building Pattern

In autonomous OpModes:
1. Define poses with position (x, y in inches) and heading (radians)
2. Create paths using `new Path(new BezierLine(startPose, endPose))`
3. Set heading interpolation: `path.setLinearHeadingInterpolation(startHeading, endHeading)`
4. Chain paths: `new PathChain(path1, path2, ...)`
5. Initialize follower in `init()`: `follower = Constants.createFollower(hardwareMap)`
6. Start following in `start()`: `follower.followPath(pathChain)`
7. Update in `loop()`: `follower.update()`

### Hardware Configuration

The robot uses a standard mecanum drivetrain with:
- 4 motors: leftFront, leftRear, rightFront, rightRear
- GoBilda Pinpoint odometry computer (name: "pinpointComputer")
- Odometry pods at: forward Y=-4", strafe X=7.0"
- IMU (commented out in current hardware setup)

### Git Workflow

Current branch: `master`
Main branch: `master`

Recent work has focused on:
- Pedro Pathing tuning (completed per commit "PEDRO TUNING IS DONE!!!")
- Autonomous path development
- Fixing heading issues

### Important Notes

- OpModes must have unique names to appear in Driver Station
- Hardware map names must match configuration on Control Hub
- The project uses FTC SDK v10.3 (as of last README merge)
- Pedro Pathing library is a third-party dependency for advanced autonomous control
- All measurements in Pedro Pathing use inches and radians
- Motor power values range from -1.0 to 1.0
