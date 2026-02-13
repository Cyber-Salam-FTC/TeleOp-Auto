# Gemini Project Context: FTC Robot Code

This document provides context for the Gemini AI assistant to understand the structure and purpose of this FTC (FIRST Tech Challenge) robot code project.

## Project Overview

This is a standard FTC robot controller project for the "Decode Season". It's a Java-based project built with Gradle. The project is organized into two main modules:

*   `FtcRobotController`: The core module from the FTC SDK.
*   `TeamCode`: The module where the team's custom robot code, including `OpModes` for TeleOp and Autonomous, is developed.

The project utilizes several third-party libraries:

*   **Pedro Pathing:** A library for advanced autonomous path following.
*   **FTControl Panels by Lazar:** A dashboard for real-time telemetry, tuning, and control.
*   **FrozenMilk:** A library that appears to provide utility functions and a custom plugin.

## Building and Running

This is an Android project that needs to be built and deployed to an FTC Robot Controller (Control Hub or a compatible Android device).

### Key Commands

*   **Build the project:**
    ```bash
    ./gradlew build
    ```
*   **Install the app on a connected device:**
    ```bash
    ./gradlew installDebug
    ```
*   **Clean the build:**
    ```bash
    ./gradlew clean
    ```

## Development Conventions

*   **Language:** The project is written in Java.
*   **Dependency Management:** Dependencies are managed by Gradle. The main dependencies are defined in `build.dependencies.gradle`, and module-specific dependencies are in `TeamCode/build.gradle`. Custom Maven repositories are configured in `settings.gradle`.
*   **Project Structure:**
    *   All team-specific code should be placed in the `TeamCode` module, within the `org.firstinspires.ftc.teamcode` package.
    *   The `pedropathing` directory seems to contain the logic for the pathfinding, with `Tuning.java` being a key file for calibration and `Constants.java` holding important parameters.
*   **OpModes:** Robot actions are organized into `OpMode` classes. These are registered for use on the Driver Station via annotations like `@TeleOp` and `@Autonomous`.

**TODO:** Review the `CONTRIBUTING.md` for more detailed contribution guidelines if available.
