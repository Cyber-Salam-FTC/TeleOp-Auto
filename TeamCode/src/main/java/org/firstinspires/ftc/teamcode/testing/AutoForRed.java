package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto For Red - Cyber Salam FTC | 26903")
public class AutoForRed extends LinearOpMode {
    int rotorPosition, delta;


    private Follower follower;
    private int pathState = 0;
    private Path currentPath;

    private DcMotor leftFront, leftRear, rightFront, rightRear, intake, rotor;
    private DcMotorEx outtake;
    private Servo door;

    private int foundID = 0;

    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private final Pose startPose = new Pose(96, 9, Math.toRadians(90));
    private final Pose SHOT_POS_1 = new Pose(96, 50, Math.toRadians(90));
    private final Pose SHOT_POS = new Pose(95, 96, Math.toRadians(225));
    private final Pose SHOT_POS_3 = new Pose(110, 83.5, Math.toRadians(0));

    private final Pose COLLECT_POS_1 = new Pose(117, 84, Math.toRadians(0));
    private final Pose COLLECT_POS_2 = new Pose(122, 84, Math.toRadians(0));
    private final Pose COLLECT_POS_3 = new Pose(134, 84, Math.toRadians(0));
    private final Pose COLLECT_POS_4 = new Pose(117, 60, Math.toRadians(0));
    private final Pose COLLECT_POS_5 = new Pose(122, 60, Math.toRadians(0));
    private final Pose COLLECT_POS_6 = new Pose(134, 60, Math.toRadians(0));
    private final Pose COLLECT_POS_7 = new Pose(117, 36, Math.toRadians(0));
    private final Pose COLLECT_POS_8 = new Pose(122, 36, Math.toRadians(0));
    private final Pose COLLECT_POS_9 = new Pose(134, 36, Math.toRadians(0));
    private final Pose PARK_POS = new Pose(105, 33, Math.toRadians(0));

    public void stateMachine() {
        switch (pathState) {
            case 0:
                door.setPosition(0);
                runShooterMechanism();
                follower.followPath(currentPath);
                pathState++;
                break;
            case 1:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_1();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_1();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_1();
                }
                sleep(500);
                shootBall();
                moveBigMovement();
                shootBall();
                moveBigMovement();
                shootBall();
                sleep(300);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 2:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_2();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_2();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_2();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 3:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_3();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_3();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_3();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 4:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_4();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_4();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_4();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 5:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_5();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_5();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_5();
                }
                shootBall();
                sleep(2000);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 6:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_6();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_6();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_6();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 7:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_7();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_7();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_7();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 8:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_8();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_8();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_8();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 9:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_9();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_9();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_9();
                }
                shootBall();
                sleep(2000);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 10:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_10();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_10();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_10();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 11:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_11();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_11();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_11();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 12:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_12();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_12();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_12();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 13:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_13();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_13();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_13();
                }
                shootBall();
                sleep(2000);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 14:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_14();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathGPP_14();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathGPP_14();
                }
                sleep(2500);
                follower.followPath(currentPath);
                pathState++;
                break;
            case 15:
                pathState++;
                break;
            default:
                stop();
                break;
        }
    }

    final int[] ROTOR_PRESETS = {0, 48, 96, 144, 192, 240};
    private final int DELTA = 48;
    private int currentRotorPosition = 0;
    int currentPresetIndex = 0;

    public void runOpMode() {
        boolean targetFound = false;

        delta = 48;



        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        door = hardwareMap.get(Servo.class, "door");
        rotor = hardwareMap.get(DcMotor.class, "rotor");


        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        rotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotor.setDirection(DcMotor.Direction.REVERSE);
        rotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotor.setTargetPosition(0);
        rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        door.setDirection(Servo.Direction.REVERSE);

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            telemetry.addData("STATUS", "Running WITHOUT Camera Detection!");
            telemetry.update();
            visionPortal = null;
        }

        telemetry.addData("Status", "Ready. Waiting for Start.");
        telemetry.update();

        door.setPosition(0);
        rotorPosition = 0;

        waitForStart();


        if (USE_WEBCAM && visionPortal != null) {
            setManualExposure(6, 250);
        }

        if (visionPortal != null) {
            while (opModeIsActive() && !targetFound && getRuntime() < 3.0) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (detection.id == GPP_TAG_ID) {
                            currentPath = buildPathGPP_1();
                            targetFound = true;
                            foundID = GPP_TAG_ID;
                            break;
                        } else if (detection.id == PGP_TAG_ID) {
                            currentPath = buildPathGPP_1();
                            targetFound = true;
                            foundID = PGP_TAG_ID;
                            break;
                        } else if (detection.id == PPG_TAG_ID) {
                            currentPath = buildPathGPP_1();
                            targetFound = true;
                            foundID = PPG_TAG_ID;
                            break;
                        }
                    }
                }


                telemetry.addData("Tag Found", targetFound);
                telemetry.addData("Found ID", foundID);
                telemetry.update();
                sleep(20);
            }
        }

        if (!targetFound) {
            currentPath = buildPathGPP_1();
            foundID = GPP_TAG_ID;
        }

        runShooterMechanism();
        stateMachine();

        while (opModeIsActive()) {
            intake.setPower(1);
            outtake.setVelocity(1800);

            follower.update();


            if (pathState > 0 && pathState < 15) {
                if (!follower.isBusy()) {
                    stateMachine();

                }
            } else if (pathState >= 5) {
                break;
            }

            updateTelemetry();

            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }

        updateTelemetry();

    }

    private Path buildPathGPP_1() {
        Path path = new Path(new BezierLine(startPose, SHOT_POS));
        path.setLinearHeadingInterpolation(startPose.getHeading(), SHOT_POS.getHeading());
        return path;
    }

    private Path buildPathGPP_2() {
        Path path = new Path(new BezierLine(SHOT_POS, COLLECT_POS_1));
        path.setLinearHeadingInterpolation(SHOT_POS.getHeading(), COLLECT_POS_1.getHeading());
        return path;
    }

    private Path buildPathGPP_3() {
        Path path = new Path(new BezierLine(COLLECT_POS_1, COLLECT_POS_2));
        path.setLinearHeadingInterpolation(COLLECT_POS_1.getHeading(), COLLECT_POS_2.getHeading());
        return path;
    }

    private Path buildPathGPP_4() {
        Path path = new Path(new BezierLine(COLLECT_POS_2, COLLECT_POS_3));
        path.setLinearHeadingInterpolation(COLLECT_POS_2.getHeading(), COLLECT_POS_3.getHeading());
        moveBigMovement();
        return path;
    }

    private Path buildPathGPP_5() {
        Path path = new Path(new BezierLine(COLLECT_POS_3, SHOT_POS));
        path.setLinearHeadingInterpolation(COLLECT_POS_3.getHeading(), SHOT_POS.getHeading());
        return path;
    }

    private Path buildPathGPP_6() {
        Path path = new Path(new BezierLine(SHOT_POS, COLLECT_POS_4));
        path.setLinearHeadingInterpolation(SHOT_POS.getHeading(), COLLECT_POS_4.getHeading());
        return path;
    }
    private Path buildPathGPP_7() {
        Path path = new Path(new BezierLine(COLLECT_POS_4, COLLECT_POS_5));
        path.setLinearHeadingInterpolation(COLLECT_POS_4.getHeading(), COLLECT_POS_5.getHeading());
        return path;
    }
    private Path buildPathGPP_8() {
        Path path = new Path(new BezierLine(COLLECT_POS_5, COLLECT_POS_6));
        path.setLinearHeadingInterpolation(COLLECT_POS_5.getHeading(), COLLECT_POS_6.getHeading());
        return path;
    }

    private Path buildPathGPP_9() {
        Path path = new Path(new BezierLine(COLLECT_POS_6, SHOT_POS));
        path.setLinearHeadingInterpolation(COLLECT_POS_6.getHeading(), SHOT_POS.getHeading());
        return path;
    }

    private Path buildPathGPP_10() {
        Path path = new Path(new BezierLine(SHOT_POS, COLLECT_POS_7));
        path.setLinearHeadingInterpolation(SHOT_POS.getHeading(), COLLECT_POS_7.getHeading());
        moveRotor(true, false);
        return path;
    }

    private Path buildPathGPP_11() {
        Path path = new Path(new BezierLine(COLLECT_POS_7, COLLECT_POS_8));
        path.setLinearHeadingInterpolation(COLLECT_POS_7.getHeading(), COLLECT_POS_8.getHeading());
        return path;
    }

    private Path buildPathGPP_12() {
        Path path = new Path(new BezierLine(COLLECT_POS_8, COLLECT_POS_9));
        path.setLinearHeadingInterpolation(COLLECT_POS_8.getHeading(), COLLECT_POS_9.getHeading());
        return path;
    }

    private Path buildPathGPP_13() {
        Path path = new Path(new BezierLine(COLLECT_POS_9, SHOT_POS));
        path.setLinearHeadingInterpolation(COLLECT_POS_9.getHeading(), SHOT_POS.getHeading());
        return path;
    }

    private Path buildPathGPP_14() {
        Path path = new Path(new BezierLine(SHOT_POS, PARK_POS));
        path.setLinearHeadingInterpolation(SHOT_POS.getHeading(), PARK_POS.getHeading());
        return path;
    }

    private Path buildPathPGP_1() {
        Path path = new Path(new BezierLine(startPose, COLLECT_POS_4));
        path.setLinearHeadingInterpolation(startPose.getHeading(), COLLECT_POS_4.getHeading());
        return path;
    }

    private Path buildPathPGP_2() {
        Path path = new Path(new BezierLine(COLLECT_POS_4, SHOT_POS_1));
        path.setLinearHeadingInterpolation(COLLECT_POS_4.getHeading(), SHOT_POS_1.getHeading());
        return path;
    }

    private Path buildPathPGP_3() {
        Path path = new Path(new BezierLine(SHOT_POS_1, SHOT_POS_3));
        path.setLinearHeadingInterpolation(SHOT_POS_1.getHeading(), SHOT_POS_3.getHeading());
        return path;
    }

    private Path buildPathPGP_4() {
        Path path = new Path(new BezierLine(SHOT_POS_3, PARK_POS));
        path.setLinearHeadingInterpolation(SHOT_POS_3.getHeading(), PARK_POS.getHeading());
        return path;
    }

    private Path buildPathPPG_1() {
        Path path = new Path(new BezierLine(startPose, SHOT_POS_3));
        path.setLinearHeadingInterpolation(startPose.getHeading(), SHOT_POS_3.getHeading());
        return path;
    }

    private Path buildPathPPG_2() {
        Path path = new Path(new BezierLine(SHOT_POS_3, COLLECT_POS_4));
        path.setLinearHeadingInterpolation(SHOT_POS_3.getHeading(), COLLECT_POS_4.getHeading());
        return path;
    }

    private Path buildPathPPG_3() {
        Path path = new Path(new BezierLine(COLLECT_POS_4, SHOT_POS_1));
        path.setLinearHeadingInterpolation(COLLECT_POS_4.getHeading(), SHOT_POS_1.getHeading());
        return path;
    }

    private Path buildPathPPG_4() {
        Path path = new Path(new BezierLine(SHOT_POS_1, PARK_POS));
        path.setLinearHeadingInterpolation(SHOT_POS_1.getHeading(), PARK_POS.getHeading());
        return path;
    }

    private void runShooterMechanism() {
        outtake.setVelocity(1500);
    }

    private void shootBall() {
        door.setPosition(0.25);
        sleep(1000);
        door.setPosition(0);
    }

    public void moveRotorSmallTickForward() {
        rotorPosition += delta;
        if (!rotor.isBusy()) {
            rotor.setTargetPosition(rotorPosition);
            rotor.setPower(1);
            rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void moveBigMovement() {
        moveRotor(true, false);
        sleep(1000);
        moveRotor(true, false);
        sleep(1000);moveRotor(true, false);
        sleep(1000);
    }

    public void moveRotor(boolean forwards, boolean backwards) {
        
        if (forwards || backwards) {

            if (forwards) {
                currentRotorPosition += DELTA;
            }
            if (backwards) {
                currentRotorPosition -= DELTA;
            }

            rotor.setTargetPosition(currentRotorPosition);
            rotor.setPower(0.7);
        }
//        boolean prevLeftBumper = false;
//        boolean prevRightBumper = false;
//
//        int rotorPosition;
//
//        boolean leftBumper, rightBumper;
//
//        rightBumper = forwards;
//        leftBumper = backwards;
//
//        prevRightBumper = rightBumper;
//        prevLeftBumper = leftBumper;
//


//        boolean forwards = rightBumper && !prevRightBumper;
//        boolean backwards = leftBumper && !prevLeftBumper;
//
//
//        rightBumper = gamepad2.right_bumper;
//        leftBumper = gamepad2.left_bumper;
//
//        forwards = rightBumper && !prevRightBumper;
//        backwards = leftBumper && !prevLeftBumper;
//
//        rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (forwards) {
//            currentPresetIndex = (currentPresetIndex + 1) % ROTOR_PRESETS.length;
//        }
//        if (backwards) {
//            currentPresetIndex = (currentPresetIndex - 1 + ROTOR_PRESETS.length) % ROTOR_PRESETS.length;
//        }
//
//        rotorPosition = ROTOR_PRESETS[currentPresetIndex];
//
//        rotor.setTargetPosition(rotorPosition);
//        rotor.setPower(0.7);
//
//        if (currentPresetIndex == 0) {
//            rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            currentPresetIndex = 1;
//            rotorPosition = ROTOR_PRESETS[currentPresetIndex];
//            rotor.setTargetPosition(rotorPosition);
//            rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        double rotorManualPower = -gamepad2.right_stick_y * 0.3;
//
//        if (Math.abs(rotorManualPower) > 0.1) {
//            if (rotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//                rotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            rotor.setPower(rotorManualPower);
//        } else if (rotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
//            rotor.setTargetPosition(rotor.getCurrentPosition());
//            rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rotor.setPower(0.7);
//        }
//
//        prevRightBumper = rightBumper;
//        prevLeftBumper = leftBumper;


    }

    public void moveRotorBigTickForward() {
        moveRotorSmallTickForward();
        moveRotorSmallTickForward();
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        double timeLimit = 5.0;
        double startTime = getRuntime();

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() &&
                    (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) &&
                    (getRuntime() < startTime + timeLimit)) {
                sleep(20);
            }
        }

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            if (exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
                gainControl.setGain(gain);
            }
        }


    }

    public void updateTelemetry() {
        telemetry.addData("Current Preset Index", currentPresetIndex);
        telemetry.addData("Rotor Mode", rotor.getMode());
        telemetry.addData("Rotor Index", currentPresetIndex);
        telemetry.addData("Rotor Current Position", rotor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtake.getVelocity());
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Door position", door.getPosition());
        telemetry.update();
    }
}