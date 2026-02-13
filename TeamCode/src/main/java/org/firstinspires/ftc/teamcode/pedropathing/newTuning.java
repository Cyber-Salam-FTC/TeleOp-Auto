//package org.firstinspires.ftc.teamcode.pedropathing;
//
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.changes;
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawCurrent;
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawCurrentAndHistory;
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.stopRobot;
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.telemetryM;
//
//import com.bylazar.configurables.PanelsConfigurables;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.telemetry.SelectableOpMode;
//import com.pedropathing.util.PoseHistory;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Configurable
//@TeleOp(name = "New Tuning", group = "Pedro Pathing")
//public class newTuning extends SelectableOpMode {
//
//        //        For AutoPIDFTuner
//        public enum TUNING_TYPE {
//            DRIVE,
//            TRANSLATIONAL,
//            LATERAL,
//            HEADING,
//            CENTRIPETAL,
//            COMPLETED
//        }
//
//        public enum VELOCITY_TUNING_STEP {
//            FORWARD,
//            LATERAL,
//            FORWARD_ZERO_POWER,
//            LATERAL_ZERO_POWER,
//            COMPLETED
//        }
//
//        static double globalForwardVelocity;
//        static double globalLateralVelocity;
//        static double globalForwardZeroPower;
//        static double globalLateralZeroPower;
//
//        static String endMessage =
//                "Forward Velocity " + globalForwardVelocity +
//                "Strafe Velocity: " + globalLateralVelocity +
//                "Forward Zero Power Acceleration " + globalForwardZeroPower +
//                "Lateral Zero Power Acceleration" + globalLateralZeroPower;
//
//
//        public static Follower follower;
//
//        @IgnoreConfigurable
//        public static PoseHistory poseHistory;
//
//        @IgnoreConfigurable
//        public static TelemetryManager telemetryM;
//
//        public newTuning() {
//            super("Select a Tuning OpMode", s -> {
//                s.add("Automatic Localization Test", AutoLocalizationTest::new);
//                s.add("Automatic PIDF Tuner", AutoPIDFTuner::new);
//                s.add("Automatic Velocity Tuning", AutoVelocityTuner::new);
//                s.folder("Tests", p -> {
//                    p.add("Line", Line::new);
//                    p.add("Triangle", Triangle::new);
//                    p.add("Circle", Circle::new);
//                });
//            });
//        }
//
//        @Override
//        public void onSelect() {
//            if (follower == null) {
//                follower = Constants.createFollower(hardwareMap);
//                PanelsConfigurables.refreshClass(this);
//            } else {
//                follower = Constants.createFollower(hardwareMap);
//            }
//
//            follower.setStartingPose(new Pose());
//
//            poseHistory = follower.getPoseHistory();
//
//            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        }
//
//        @Override
//        public void onLog(List<String> lines) {
//        }
//
//        public static void drawCurrent() {
//            try {
//                Drawing.drawRobot(follower.getPose());
//                Drawing.sendPacket();
//            } catch (Exception e) {
//                throw new RuntimeException("Drawing failed: " + e);
//            }
//        }
//
//        public static void drawCurrentAndHistory() {
//            Drawing.drawPoseHistory(poseHistory);
//            drawCurrent();
//        }
//
//
//        /**
//         * This is an auto
//         *
//         * @author Zidan Harb - 26903 Cyber Salam
//         * @version 1.0, 1/28/2026
//         */
//
//        static class AutoPIDFTuner extends OpMode {
//
//            TUNING_TYPE step = TUNING_TYPE.DRIVE;
//
//            /**
//             * This are the initial values set in order to start the automatic tuning
//             */
//            double drive_kV, drive_kA, drive_kP = 0.01;
//            double trans_kP, trans_kD;
//            double lateral_kP, lateral_kD;
//            double heading_kP, heading_kD;
//            double centripetal_kF;
//
//            /**
//             * These are internal variables set for the tuning
//             */
//
//            private ElapsedTime stepTimer = new ElapsedTime();
//            private double maxVelocity = 0, maxAcceleration = 0, previousVelocity = 0;
//            private double highKp = 0.001;
//            private List<Double> peakTimes = new ArrayList<>();
//            private boolean isOscillating = false;
//            private double previousStepTime = 0;
//
//
//            @Override
//            public void init() {}
//
//            @Override
//            public void init_loop() {
//                telemetryM.debug("The robot will move in a plane of 10ft." + " Please make at least 10ft of space for the" +
//                        " robot to move around");
//            }
//
//            @Override
//            public void start() {
//                stepTimer.reset();
//            }
//
//            @Override
//            public void loop() {
//                follower.update();
//
//                double currentTime = stepTimer.seconds();
//
//                /**
//                 * This state machine, running on the enum set [currentStep], will handle each tuning step
//                 */
//
//                switch (step) {
//                    case DRIVE:
//                        //                Drive tuner
//                        if (runDriveTuner(currentTime)) {
//                            nextStep(TUNING_TYPE.TRANSLATIONAL);
//                        }
//                        break;
//                    case TRANSLATIONAL:
//                        //                Translational tuner
//                        if (runOscillation(0, currentTime)) {
//                            nextStep(TUNING_TYPE.LATERAL);
//                        }
//                        break;
//                    case LATERAL:
//                        //                Lateral tuner
//                        if (runOscillation(1, currentTime)) {
//                            nextStep(TUNING_TYPE.HEADING);
//                        }
//                        break;
//                    case HEADING:
//                        //                Heading tuner
//                        if (runOscillation(2, currentTime)) {
//                            nextStep(TUNING_TYPE.CENTRIPETAL);
//                        }
//                        break;
//                    case CENTRIPETAL:
//                        //                Centripetal tuner
//                        if (runCentripetal(currentTime)) {
//                            nextStep(TUNING_TYPE.COMPLETED);
//                        }
//                        break;
//                    case COMPLETED:
//                        printResults();
//                        break;
//                }
//
//
//            }
//
//            private boolean runDriveTuner(double t) {
//                /**  @param t is the time converted to milliseconds */
//
//                double currentVelocity = follower.getVelocity().getMagnitude();
//                if (currentVelocity > maxVelocity) maxVelocity = currentVelocity;
//
//                /**
//                 * Here, we calculate and tune the drive PIDF using the follower.setTeleOpDrive method
//                 */
//
//                double acceleration = (currentVelocity - previousVelocity) / 0.02;
//                if (acceleration > maxAcceleration && t < 1.0) maxAcceleration = acceleration;
//                previousVelocity = currentVelocity;
//
//                if (t < 2.0) {
//                    follower.setTeleOpDrive(1.0, 0, 0, false);
//                    return false;
//                } else {
//                    //            This is the calculation to keep the robot moving at a fixed speed
//                    drive_kV = 1.0 / maxVelocity;
//                    //            This is the calculation to add the power need to overcome the robot's weight
//                    drive_kA = 1.0 / maxAcceleration;
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    return true;
//                }
//            }
//
//            /**
//             * This boolean preforms the oscillatory test to find th kP and kV values using the Ziegler-Nichols method
//             */
//
//            private boolean runOscillation(int axis, double t) {
//                double error = (axis == 0) ? follower.getPose().getX() : (axis == 1) ? follower.getPose().getY() : follower.getPose().getHeading();
//
//                if (!isOscillating) {
//                    highKp += 0.0005;
//                    if (Math.abs(error) > 5.0) isOscillating = true;
//                    applyPower(axis, highKp * -error);
//                } else {
//                    if (Math.signum(error) != Math.signum(previousVelocity)) {
//                        peakTimes.add(t);
//                    }
//                    applyPower(axis, highKp * -error);
//                }
//
//                previousVelocity = error;
//
//                /**
//                 * Here, we stop when there is enough peaks for an average value to use
//                 */
//
//                if (peakTimes.size() >= 6 || t > 12.0) {
//                    double Tu = (peakTimes.get(peakTimes.size() - 1) - peakTimes.get(0)) / (peakTimes.size() / 2.0);
//                    double Ku = highKp;
//                    calculate(axis, Ku, Tu);
//                    return true;
//                }
//                return false;
//            }
//
//            //    Running the centripetal test
//
//            private boolean runCentripetal(double t) {
//                double radius = 30.0;
//                double v = 20.0;
//                follower.setTeleOpDrive(v * drive_kV, 0, v / radius, false);
//
//                double actualRadius = Math.hypot(follower.getPose().getX(), follower.getPose().getY());
//                if (t > 4.0) {
//                    centripetal_kF = (actualRadius - radius) / (Math.pow(v, 2) / radius);
//                    return true;
//                }
//                return false;
//            }
//
//            /**
//             * This is the ending parts of the tuning session, where we calculate all the values
//             * to end and have all the values ready to be printed
//             */
//            private void calculate(int axis, double Ku, double Tu) {
//                double p = 0.6 * Ku;
//                double d = (p * Tu) / 8.0;
//                if (axis == 0) {
//                    trans_kP = p;
//                    trans_kD = d;
//                }
//                if (axis == 1) {
//                    lateral_kP = p;
//                    lateral_kD = d;
//                }
//                if (axis == 2) {
//                    heading_kP = p;
//                    heading_kD = d;
//                }
//            }
//
//            private void applyPower(int axis, double power) {
//                if (axis == 0) follower.setTeleOpDrive(power, 0, 0, false);
//                if (axis == 1) follower.setTeleOpDrive(0, power, 0, false);
//                if (axis == 2) follower.setTeleOpDrive(0, 0, power, false);
//            }
//
//            //    Resetting everything to get ready to run upcoming tests
//            private void nextStep(TUNING_TYPE next) {
//                follower.setTeleOpDrive(0, 0, 0, false);
//                step = next;
//                stepTimer.reset(); // This replaces stepStartTime manual assignment
//                highKp = 0.001;
//                peakTimes.clear();
//                isOscillating = false;
//                maxVelocity = 0;
//                maxAcceleration = 0;
//                previousVelocity = 0;
//            }
//
//            /**
//             * Finally, the end. We print out all the PIDF values we have calculated.
//             * This completes the session.
//             */
//
//            private void printResults() {
//                telemetry.addLine("=== TUNING COMPLETE ===");
//                telemetry.addData("Drive FF", "kV: %.5f, kA: %.5f", drive_kV, drive_kA);
//                telemetry.addData("Translational", "kP: %.4f, kD: %.4f", trans_kP, trans_kD);
//                telemetry.addData("Lateral", "kP: %.4f, kD: %.4f", lateral_kP, lateral_kD);
//                telemetry.addData("Heading", "kP: %.4f, kD: %.4f", heading_kP, heading_kD);
//                telemetry.addData("Centripetal", "kF: %.5f", centripetal_kF);
//                telemetry.update();
//            }
//
//        }
//
//        static class AutoVelocityTuner extends OpMode {
//
//            VELOCITY_TUNING_STEP velocityStep = VELOCITY_TUNING_STEP.FORWARD;
//
//            private ElapsedTime stepTimer = new ElapsedTime();
//
//            @Override
//            public void init() {}
//
//            @Override
//            public void init_loop() {
//                telemetryM.debug("The robot will move in a plane of 10ft." +
//                        " Please make at least 10ft of space for the" +
//                        " robot to move around");
//            }
//
//
//
//            @Override
//            public void loop() {
//
//                switch(velocityStep) {
//                    case FORWARD:
//                        forwardTuner();
//                        nextStep(VELOCITY_TUNING_STEP.LATERAL);
//                        break;
//                    case LATERAL:
//                        lateralTuner();
//                        nextStep(VELOCITY_TUNING_STEP.FORWARD_ZERO_POWER);
//                        break;
//                    case FORWARD_ZERO_POWER:
//                        forwardZeroPowerTuner();
//                        nextStep(VELOCITY_TUNING_STEP.LATERAL_ZERO_POWER);
//                        break;
//                    case LATERAL_ZERO_POWER:
//                        lateralZeroPowerTuner();
//                        nextStep(VELOCITY_TUNING_STEP.COMPLETED);
//                        break;
//                    default:
//                        printResults();
//                }
//            }
//
//            public void forwardTuner() {
//                ForwardTuner forwardtuner = new ForwardTuner();
//
//                forwardtuner.hardwareMap = this.hardwareMap;
//                forwardtuner.telemetry = this.telemetry;
//                forwardtuner.gamepad1 = this.gamepad1;
//                forwardtuner.gamepad2 = this.gamepad2;
//
//                forwardtuner.init();
//                forwardtuner.init_loop();
//
//                forwardtuner.start();
//                forwardtuner.loop();
//
//            }
//
//            public void lateralTuner() {
//                LateralVelocityTuner lateralTuner = new LateralVelocityTuner();
//
//                lateralTuner.hardwareMap = this.hardwareMap;
//                lateralTuner.telemetry = this.telemetry;
//                lateralTuner.gamepad1 = this.gamepad1;
//                lateralTuner.gamepad2 = this.gamepad2;
//
//                lateralTuner.init();
//                lateralTuner.init_loop();
//
//                lateralTuner.start();
//                lateralTuner.loop();
//
//            }
//
//            public void forwardZeroPowerTuner() {
//                ForwardZeroPower forwardZeroPowerTuner = new ForwardZeroPower();
//
//                forwardZeroPowerTuner.hardwareMap = this.hardwareMap;
//                forwardZeroPowerTuner.telemetry = this.telemetry;
//                forwardZeroPowerTuner.gamepad1 = this.gamepad1;
//                forwardZeroPowerTuner.gamepad2 = this.gamepad2;
//
//                forwardZeroPowerTuner.init();
//                forwardZeroPowerTuner.init_loop();
//
//                forwardZeroPowerTuner.start();
//                forwardZeroPowerTuner.loop();
//
//            }
//
//            public void lateralZeroPowerTuner() {
//                LateralZeroPower lateralZeroPowerTuner = new LateralZeroPower();
//
//                lateralZeroPowerTuner.hardwareMap = this.hardwareMap;
//                lateralZeroPowerTuner.telemetry = this.telemetry;
//                lateralZeroPowerTuner.gamepad1 = this.gamepad1;
//                lateralZeroPowerTuner.gamepad2 = this.gamepad2;
//
//                lateralZeroPowerTuner.init();
//                lateralZeroPowerTuner.init_loop();
//
//                lateralZeroPowerTuner.start();
//                lateralZeroPowerTuner.loop();
//
//            }
//
//            public void nextStep(VELOCITY_TUNING_STEP next) {
//                velocityStep = next;
//            }
//
//            public void printResults() {
//
//            }
//        }
//
//        /**
//         * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
//         * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
//         * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
//         * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
//         * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
//         * and average the results. Then, input the multiplier into the forward ticks to inches in your
//         * localizer of choice.
//         *
//         * @author Anyi Lin - 10158 Scott's Bots
//         * @author Baron Henderson - 20077 The Indubitables
//         * @author Zidan Harb - 26903 Cyber Salam
//         * @version 1.1, 2/3/2026
//         */
//        static class ForwardTuner extends OpMode {
//            double DISTANCE = 48;
//
//            @Override
//            public void init() {
//                follower.update();
//                drawCurrent();
//            }
//
//            /** This initializes the PoseUpdater as well as the Panels telemetry. */
//            @Override
//            public void init_loop() {
//                telemetryM.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
//                telemetryM.update(telemetry);
//                drawCurrent();
//            }
//
//            /**
//             * This updates the robot's pose estimate, and updates the Panels telemetry with the
//             * calculated multiplier and draws the robot.
//             */
//            @Override
//            public void loop() {
//                follower.update();
//
//                telemetryM.debug("Distance Moved: " + follower.getPose().getX());
//                telemetryM.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
//                telemetryM.debug("Multiplier: " + (DISTANCE / (follower.getPose().getX() / follower.getPoseTracker().getLocalizer().getForwardMultiplier())));
//                telemetryM.update(telemetry);
//
//                drawCurrentAndHistory();
//            }
//
//
//        }
//
//        /**
//         * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot right at max
//         * power until it reaches some specified distance. It records the most recent velocities, and on
//         * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
//         * recommended to run this multiple times on a full battery to get the best results. What this does
//         * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
//         * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
//         * more accurate following.
//         *
//         * @author Anyi Lin - 10158 Scott's Bots
//         * @author Aaron Yang - 10158 Scott's Bots
//         * @author Harrison Womack - 10158 Scott's Bots
//         * @author Baron Henderson - 20077 The Indubitables
//         * @author Zidan Harb - 26903 Cyber Salam
//         * @version 1.1, 2/5/2026
//         */
//        static class LateralVelocityTuner extends OpMode {
//            private final ArrayList<Double> velocities = new ArrayList<>();
//
//            double DISTANCE = 48;
//            double RECORD_NUMBER = 10;
//
//            private boolean end;
//
//            @Override
//            public void init() {}
//
//            /**
//             * This initializes the drive motors as well as the cache of velocities and the Panels
//             * telemetryM.
//             */
//            @Override
//            public void init_loop() {
//                telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the right.");
//                telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
//                telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.");
//                telemetryM.debug("Press B on Gamepad 1 to stop.");
//                telemetryM.update(telemetry);
//
//                follower.update();
//                drawCurrent();
//            }
//
//            /** This starts the OpMode by setting the drive motors to run right at full power. */
//            @Override
//            public void start() {
//                for (int i = 0; i < RECORD_NUMBER; i++) {
//                    velocities.add(0.0);
//                }
//                follower.startTeleopDrive(true);
//                follower.update();
//            }
//
//            /**
//             * This runs the OpMode. At any point during the running of the OpMode, pressing B on
//             * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
//             * velocities, and when the robot has run sideways enough, these last velocities recorded are
//             * averaged and printed.
//             */
//            @Override
//            public void loop() {
//                if (gamepad1.bWasPressed()) {
//                    stopRobot();
//                    requestOpModeStop();
//                }
//
//                follower.update();
//                drawCurrentAndHistory();
//
//                if (!end) {
//                    if (Math.abs(follower.getPose().getY()) > DISTANCE) {
//                        end = true;
//                        stopRobot();
//                    } else {
//                        follower.setTeleOpDrive(0,1,0,true);
//                        double currentVelocity = Math.abs(follower.getVelocity().dot(new Vector(1, Math.PI / 2)));
//                        velocities.add(currentVelocity);
//                        velocities.remove(0);
//                    }
//                } else {
//                    stopRobot();
//                    double average = 0;
//                    for (double velocity : velocities) {
//                        average += velocity;
//                    }
//                    average /= velocities.size();
//
////                    telemetryM.debug("Strafe Velocity: " + average);
////                    telemetryM.debug("\n");
//
//                    globalLateralVelocity = average;
//
//                    globalForwardVelocity = average;
//                    telemetryM.debug("Press A to set the Lateral Velocity temporarily (while robot remains on).");
//                    telemetryM.update(telemetry);
//
//                    if (gamepad1.aWasPressed()) {
//                        follower.setYVelocity(average);
//                        String message = "YMovement: " + average;
//                        changes.add(message);
//                    }
//                }
//            }
//        }
//
//        /**
//         * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
//         * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
//         * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
//         * stops. The accelerations across the entire time the robot is slowing down is then averaged and
//         * that number is then printed. This is used to determine how the robot will decelerate in the
//         * forward direction when power is cut, making the estimations used in the calculations for the
//         * drive Vector more accurate and giving better braking at the end of Paths.
//         *
//         * @author Anyi Lin - 10158 Scott's Bots
//         * @author Baron Henderson - 20077 The Indubitables
//         * @author Aaron Yang - 10158 Scott's Bots
//         * @author Harrison Womack - 10158 Scott's Bots
//         * @author Zidan Harb - 26903 Cyber Salam
//         * @version 1.1, 2/5/2026
//         */
//
//        static class ForwardZeroPower extends OpMode {
//            private final ArrayList<Double> accelerations = new ArrayList<>();
//            double VELOCITY = 30;
//
//            private double previousVelocity;
//            private long previousTimeNano;
//
//            private boolean stopping;
//            private boolean end;
//
//            @Override
//            public void init() {}
//
//            /** This initializes the drive motors as well as the Panels telemetryM. */
//            @Override
//            public void init_loop() {
//                telemetryM.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
//                telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
//                telemetryM.debug("Make sure you have enough room.");
//                telemetryM.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
//                telemetryM.debug("Press B on Gamepad 1 to stop.");
//                telemetryM.update(telemetry);
//                follower.update();
//                drawCurrent();
//            }
//
//            /** This starts the OpMode by setting the drive motors to run forward at full power. */
//            @Override
//            public void start() {
//                follower.startTeleopDrive(false);
//                follower.update();
//                follower.setTeleOpDrive(1,0,0,true);
//            }
//
//            /**
//             * This runs the OpMode. At any point during the running of the OpMode, pressing B on
//             * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
//             * record its deceleration / negative acceleration until it stops. Then, it will average all the
//             * recorded deceleration / negative acceleration and print that value.
//             */
//            @Override
//            public void loop() {
//                if (gamepad1.bWasPressed()) {
//                    stopRobot();
//                    requestOpModeStop();
//                }
//
//                follower.update();
//                drawCurrentAndHistory();
//
//                Vector heading = new Vector(1.0, follower.getPose().getHeading());
//                if (!end) {
//                    if (!stopping) {
//                        if (follower.getVelocity().dot(heading) > VELOCITY) {
//                            previousVelocity = follower.getVelocity().dot(heading);
//                            previousTimeNano = System.nanoTime();
//                            stopping = true;
//                            follower.setTeleOpDrive(0,0,0,true);
//                        }
//                    } else {
//                        double currentVelocity = follower.getVelocity().dot(heading);
//                        accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
//                        previousVelocity = currentVelocity;
//                        previousTimeNano = System.nanoTime();
//                        if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
//                            end = true;
//                        }
//                    }
//                } else {
//                    double average = 0;
//                    for (double acceleration : accelerations) {
//                        average += acceleration;
//                    }
//                    average /= accelerations.size();
//
////                    telemetryM.debug("Forward Zero Power Acceleration (Deceleration): " + average);
////                    telemetryM.debug("\n");
////                    telemetryM.debug("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).");
//
//                    globalForwardZeroPower = average;
//                    telemetryM.update(telemetry);
//
//                    if (gamepad1.aWasPressed()) {
//                        follower.getConstants().setForwardZeroPowerAcceleration(average);
//                        String message = "Forward Zero Power Acceleration: " + average;
//                        changes.add(message);
//                    }
//                }
//            }
//        }
//        /**
//         * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
//         * to the right until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
//         * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
//         * stops. The accelerations across the entire time the robot is slowing down is then averaged and
//         * that number is then printed. This is used to determine how the robot will decelerate in the
//         * forward direction when power is cut, making the estimations used in the calculations for the
//         * drive Vector more accurate and giving better braking at the end of Paths.
//         *
//         * @author Anyi Lin - 10158 Scott's Bots
//         * @author Aaron Yang - 10158 Scott's Bots
//         * @author Harrison Womack - 10158 Scott's Bots
//         * @author Baron Henderson - 20077 The Indubitables
//         * @author Zidan Harb - 26903 Cyber Salam
//         * @version 1.1, 2/5/2026
//         */
//        static class LateralZeroPower extends OpMode {
//            private final ArrayList<Double> accelerations = new ArrayList<>();
//            double VELOCITY = 30;
//            private double previousVelocity;
//            private long previousTimeNano;
//            private boolean stopping;
//            private boolean end;
//
//            @Override
//            public void init() {}
//
//            /** This initializes the drive motors as well as the Panels telemetry. */
//            @Override
//            public void init_loop() {
//                telemetryM.debug("The robot will run to the right until it reaches " + VELOCITY + " inches per second.");
//                telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
//                telemetryM.debug("Make sure you have enough room.");
//                telemetryM.debug("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.");
//                telemetryM.debug("Press B on game pad 1 to stop.");
//                telemetryM.update(telemetry);
//                follower.update();
//                drawCurrent();
//            }
//
//            /** This starts the OpMode by setting the drive motors to run forward at full power. */
//            @Override
//            public void start() {
//                follower.startTeleopDrive(false);
//                follower.update();
//                follower.setTeleOpDrive(0,1,0,true);
//            }
//
//            /**
//             * This runs the OpMode. At any point during the running of the OpMode, pressing B on
//             * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
//             * record its deceleration / negative acceleration until it stops. Then, it will average all the
//             * recorded deceleration / negative acceleration and print that value.
//             */
//            @Override
//            public void loop() {
//                if (gamepad1.bWasPressed()) {
//                    stopRobot();
//                    requestOpModeStop();
//                }
//
//                follower.update();
//                drawCurrentAndHistory();
//
//                Vector heading = new Vector(1.0, follower.getPose().getHeading() - Math.PI / 2);
//                if (!end) {
//                    if (!stopping) {
//                        if (Math.abs(follower.getVelocity().dot(heading)) > VELOCITY) {
//                            previousVelocity = Math.abs(follower.getVelocity().dot(heading));
//                            previousTimeNano = System.nanoTime();
//                            stopping = true;
//                            follower.setTeleOpDrive(0,0,0,true);
//                        }
//                    } else {
//                        double currentVelocity = Math.abs(follower.getVelocity().dot(heading));
//                        accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
//                        previousVelocity = currentVelocity;
//                        previousTimeNano = System.nanoTime();
//                        if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
//                            end = true;
//                        }
//                    }
//                } else {
//                    double average = 0;
//                    for (double acceleration : accelerations) {
//                        average += acceleration;
//                    }
//                    average /= accelerations.size();
//
////                    telemetryM.debug("Lateral Zero Power Acceleration (Deceleration): " + average);
////                    telemetryM.debug("\n");
////                    telemetryM.debug("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).");
//                    telemetryM.update(telemetry);
//
//                    globalLateralZeroPower = average;
//
//                    if (gamepad1.aWasPressed()) {
//                        follower.getConstants().setLateralZeroPowerAcceleration(average);
//                        String message = "Lateral Zero Power Acceleration: " + average;
//                        changes.add(message);
//                    }
//                }
//            }
//        }
//
//    /**
//     * This is the AutoLocalizationTest OpMode. This is an automatic tuner to ensure your encoder directions are correct using the
//     * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as drawing the robot in Panels.
//     * You should use this to check the robot's localization and make sure the encoder directions are correct.
//     *
//     * @author Zidan Harb - 26903 Cyber Salam
//     * @version 1.0, 1/25/2026
//     */
//
//    class AutoLocalizationTest extends OpMode {
//
//        Boolean xCorrect;
//        Boolean yCorrect;
//        boolean movingForward = false;
//        boolean movingSide = false;
//
//        double startEncoderValue;
//
//        @Override
//        public void init() {}
//
//        @Override
//        public void init_loop() {
//            telemetryM.debug("On start, the robot will move 2000 ticks (measured based on chassis type)" +
//                    "forward and left. Make sure you have enough room for the robot to move around.");
//            telemetryM.update(telemetry);
//            telemetryM.update();
//            follower.update();
//            drawCurrent();
//        }
//
//        @Override
//        public void start() {
//            startEncoderValue = follower.getPose().getX();
//            movingForward = true;
//        }
//
//        @Override
//        public void loop() {
//            follower.update();
//
//            double x = follower.getPose().getX();
//            double y = follower.getPose().getY();
//
//            int TICKS_TO_MOVE = 2000;
//            if (movingForward) {
//                if (Math.abs(x - startEncoderValue) < TICKS_TO_MOVE) {
//                    follower.setTeleOpDrive(0.5, 0, 0, false);
//                } else {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    xCorrect = (x > startEncoderValue);
//                    movingForward = false;
//                    movingSide = true;
//                    startEncoderValue = y;
//                }
//            } else if (movingSide) {
//                if (Math.abs(y - startEncoderValue) < TICKS_TO_MOVE) {
//                    follower.setTeleOpDrive(0, 0.5, 0, false);
//                } else {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    yCorrect = (y > startEncoderValue);
//                    movingSide = false;
//                }
//            }
//
//            telemetryM.debug("x: " + x);
//            telemetryM.debug("y: " + y);
//            telemetryM.debug("Forward Encoder Direction Correct: " + xCorrect);
//            telemetryM.debug("Strafe Encoder Direction Correct: " + yCorrect);
//            telemetryM.debug("\n");
//            telemetryM.debug("If there are any null values, check your wiring and configuration again!");
//            telemetryM.update(telemetry);
//        }
//    }
//
//    /**
//     * This is the Line Test Tuner OpMode. It will drive the robot forward and back
//     * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
//     *
//     * @author Baron Henderson - 20077 The Indubitables
//     * @author Anyi Lin - 10158 Scott's Bots
//     * @author Aaron Yang - 10158 Scott's Bots
//     * @author Harrison Womack - 10158 Scott's Bots
//     * @version 1.0, 3/12/2024
//     */
//    class Line extends OpMode {
//        public double DISTANCE = 40;
//        private boolean forward = true;
//
//        private Path forwards;
//        private Path backwards;
//
//        @Override
//        public void init() {}
//
//        /** This initializes the Follower and creates the forward and backward Paths. */
//        @Override
//        public void init_loop() {
//            telemetryM.debug("This will activate all the PIDF(s)");
//            telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
//            telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
//            telemetryM.update(telemetry);
//            follower.update();
//            drawCurrent();
//        }
//
//        @Override
//        public void start() {
//            follower.activateAllPIDFs();
//            forwards = new Path(new BezierLine(new Pose(0,0), new Pose(DISTANCE,0)));
//            forwards.setConstantHeadingInterpolation(0);
//            backwards = new Path(new BezierLine(new Pose(DISTANCE,0), new Pose(0,0)));
//            backwards.setConstantHeadingInterpolation(0);
//            follower.followPath(forwards);
//        }
//
//        /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
//        @Override
//        public void loop() {
//            follower.update();
//            drawCurrentAndHistory();
//
//            if (!follower.isBusy()) {
//                if (forward) {
//                    forward = false;
//                    follower.followPath(backwards);
//                } else {
//                    forward = true;
//                    follower.followPath(forwards);
//                }
//            }
//
//            telemetryM.debug("Driving Forward?: " + forward);
//            telemetryM.update(telemetry);
//        }
//    }
//
//    /**
//     * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
//     * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
//     * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
//     * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
//     * centripetal Vector.
//     *
//     * @author Baron Henderson - 20077 The Indubitables
//     * @author Anyi Lin - 10158 Scott's Bots
//     * @author Aaron Yang - 10158 Scott's Bots
//     * @author Harrison Womack - 10158 Scott's Bots
//     * @version 1.0, 3/13/2024
//     */
//    class CentripetalTuner extends OpMode {
//        public double DISTANCE = 20;
//        private boolean forward = true;
//
//        private Path forwards;
//        private Path backwards;
//
//        @Override
//        public void init() {}
//
//        /**
//         * This initializes the Follower and creates the forward and backward Paths.
//         * Additionally, this initializes the Panels telemetry.
//         */
//        @Override
//        public void init_loop() {
//            telemetryM.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.");
//            telemetryM.debug("The robot will go continuously along the path.");
//            telemetryM.debug("Make sure you have enough room.");
//            telemetryM.update(telemetry);
//            follower.update();
//            drawCurrent();
//        }
//
//        @Override
//        public void start() {
//            follower.activateAllPIDFs();
//            forwards = new Path(new BezierCurve(new Pose(), new Pose(Math.abs(DISTANCE),0), new Pose(Math.abs(DISTANCE),DISTANCE)));
//            backwards = new Path(new BezierCurve(new Pose(Math.abs(DISTANCE),DISTANCE), new Pose(Math.abs(DISTANCE),0), new Pose(0,0)));
//
//            backwards.setTangentHeadingInterpolation();
//            backwards.reverseHeadingInterpolation();
//
//            follower.followPath(forwards);
//        }
//
//        /**
//         * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//         * the Telemetry, as well as the Panels.
//         */
//        @Override
//        public void loop() {
//            follower.update();
//            drawCurrentAndHistory();
//            if (!follower.isBusy()) {
//                if (forward) {
//                    forward = false;
//                    follower.followPath(backwards);
//                } else {
//                    forward = true;
//                    follower.followPath(forwards);
//                }
//            }
//
//            telemetryM.debug("Driving away from the origin along the curve?: " + forward);
//            telemetryM.update(telemetry);
//        }
//    }
//
//    /**
//     * This is the Triangle autonomous OpMode.
//     * It runs the robot in a triangle, with the starting point being the bottom-middle point.
//     *
//     * @author Baron Henderson - 20077 The Indubitables
//     * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
//     * @version 1.0, 12/30/2024
//     */
//    class Triangle extends OpMode {
//
//        private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
//        private final Pose interPose = new Pose(24, -24, Math.toRadians(90));
//        private final Pose endPose = new Pose(24, 24, Math.toRadians(45));
//
//        private PathChain triangle;
//
//        /**
//         * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//         * the Telemetry, as well as the Panels.
//         */
//        @Override
//        public void loop() {
//            follower.update();
//            drawCurrentAndHistory();
//
//            if (follower.atParametricEnd()) {
//                follower.followPath(triangle, true);
//            }
//        }
//
//        @Override
//        public void init() {}
//
//        @Override
//        public void init_loop() {
//            telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
//            telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
//            telemetryM.update(telemetry);
//            follower.update();
//            drawCurrent();
//        }
//
//        /** Creates the PathChain for the "triangle".*/
//        @Override
//        public void start() {
//            follower.setStartingPose(startPose);
//
//            triangle = follower.pathBuilder()
//                    .addPath(new BezierLine(startPose, interPose))
//                    .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
//                    .addPath(new BezierLine(interPose, endPose))
//                    .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
//                    .addPath(new BezierLine(endPose, startPose))
//                    .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
//                    .build();
//
//            follower.followPath(triangle);
//        }
//    }
//
//    /**
//     * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
//     * a circle, but some Bezier curves that have control points set essentially in a square. However,
//     * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
//     * heading is to be expected.
//     *
//     * @author Anyi Lin - 10158 Scott's Bots
//     * @author Aaron Yang - 10158 Scott's Bots
//     * @author Harrison Womack - 10158 Scott's Bots
//     * @version 1.0, 3/12/2024
//     */
//    class Circle extends OpMode {
//        public double RADIUS = 10;
//        private PathChain circle;
//
//        public void start() {
//            circle = follower.pathBuilder()
//                    .addPath(new BezierCurve(new Pose(0, 0), new Pose(RADIUS, 0), new Pose(RADIUS, RADIUS)))
//                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, RADIUS))
//                    .addPath(new BezierCurve(new Pose(RADIUS, RADIUS), new Pose(RADIUS, 2 * RADIUS), new Pose(0, 2 * RADIUS)))
//                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, RADIUS))
//                    .addPath(new BezierCurve(new Pose(0, 2 * RADIUS), new Pose(-RADIUS, 2 * RADIUS), new Pose(-RADIUS, RADIUS)))
//                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, RADIUS))
//                    .addPath(new BezierCurve(new Pose(-RADIUS, RADIUS), new Pose(-RADIUS, 0), new Pose(0, 0)))
//                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, RADIUS))
//                    .build();
//            follower.followPath(circle);
//        }
//
//        @Override
//        public void init_loop() {
//            telemetryM.debug("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge. ");
//            telemetryM.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.");
//            telemetryM.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.");
//            telemetryM.update(telemetry);
//            follower.update();
//            drawCurrent();
//        }
//
//        @Override
//        public void init() {}
//
//        /**
//         * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//         * the Telemetry, as well as the FTC Dashboard.
//         */
//        @Override
//        public void loop() {
//            follower.update();
//            drawCurrentAndHistory();
//
//            if (follower.atParametricEnd()) {
//                follower.followPath(circle);
//            }
//        }
//    }
//}