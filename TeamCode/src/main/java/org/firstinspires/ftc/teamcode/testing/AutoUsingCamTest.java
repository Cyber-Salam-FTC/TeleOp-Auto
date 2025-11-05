package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "Auto Using Camera - Cyber Salam FTC | 26903")
public class AutoUsingCamTest extends LinearOpMode {
    private Follower follower;
    private int pathState = 0;
    private Path currentPath;

    private DcMotor leftFront, leftRear, rightFront, rightRear, intake;
    private Servo door;

    private int foundID = 0;

    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private final Pose startPose = new Pose(96, 8, Math.toRadians(90));
    private final Pose SHOT_POS_1 = new Pose(96, 50, Math.toRadians(90));
    private final Pose COLLECT_POS_2 = new Pose(96, 96, Math.toRadians(45));
    private final Pose SHOT_POS_3 = new Pose(110, 83.5, Math.toRadians(0));
    private final Pose COLLECT_POS_4 = new Pose(35, 83.5, Math.toRadians(180));
    private final Pose SHOT_POS_5 = new Pose(24, 83.5, Math.toRadians(180));
    private final Pose PARK_POS = new Pose(96, 96, Math.toRadians(45));

    public void stateMachine() {
        switch (pathState) {
            case 0:
                runShooterMechanism();
                follower.followPath(currentPath);
                pathState++;
                break;
            case 1:
                shootBall();
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_2();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathPGP_2();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathPPG_2();
                }
                follower.followPath(currentPath);
                pathState++;
                break;
            case 2:
                if (foundID == GPP_TAG_ID) {
                    shootBall();
                    currentPath = buildPathGPP_3();
                } else if (foundID == PGP_TAG_ID) {
                    shootBall();
                    currentPath = buildPathPGP_3();
                } else if (foundID == PPG_TAG_ID) {
                    currentPath = buildPathPPG_3();
                }
                follower.followPath(currentPath);
                pathState++;
                break;
            case 3:
                if (foundID == GPP_TAG_ID) {
                    currentPath = buildPathGPP_4();
                } else if (foundID == PGP_TAG_ID) {
                    currentPath = buildPathPGP_4();
                } else if (foundID == PPG_TAG_ID) {
                    shootBall();
                    currentPath = buildPathPPG_4();
                }
                follower.followPath(currentPath);
                pathState++;
                break;
            case 4:
                shootBall();
                pathState++;
                break;
            default:
                stop();
                break;
        }
    }

    public void runOpMode() {
        boolean targetFound = false;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        door = hardwareMap.get(Servo.class, "door");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

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
                            currentPath = buildPathPGP_1();
                            targetFound = true;
                            foundID = PGP_TAG_ID;
                            break;
                        } else if (detection.id == PPG_TAG_ID) {
                            currentPath = buildPathPPG_1();
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

        stateMachine();

        while (opModeIsActive()) {
            intake.setPower(1);

            follower.update();


            if (pathState > 0 && pathState < 5) {
                if (!follower.isBusy()) {
                    stateMachine();
                }
            } else if (pathState >= 5) {
                break;
            }

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private Path buildPathGPP_1() {
        Path path = new Path(new BezierLine(startPose, SHOT_POS_1));
        path.setLinearHeadingInterpolation(startPose.getHeading(), SHOT_POS_1.getHeading());
        return path;
    }

    private Path buildPathGPP_2() {
        Path path = new Path(new BezierLine(SHOT_POS_1, COLLECT_POS_4));
        path.setLinearHeadingInterpolation(SHOT_POS_1.getHeading(), COLLECT_POS_4.getHeading());
        return path;
    }

    private Path buildPathGPP_3() {
        Path path = new Path(new BezierLine(COLLECT_POS_4, SHOT_POS_3));
        path.setLinearHeadingInterpolation(COLLECT_POS_4.getHeading(), SHOT_POS_3.getHeading());
        return path;
    }

    private Path buildPathGPP_4() {
        Path path = new Path(new BezierLine(SHOT_POS_3, PARK_POS));
        path.setLinearHeadingInterpolation(SHOT_POS_3.getHeading(), PARK_POS.getHeading());
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
//        run outtake
    }

    private void shootBall() {
        door.setPosition(0.15);
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
}