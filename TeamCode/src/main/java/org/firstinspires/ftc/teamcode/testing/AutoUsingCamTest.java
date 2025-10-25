package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private Path path1, path2, path3, path4, path5, path6, path7, path8, path9;
    private Path PGPpath, PPGpath;

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private int foundID = 0;

    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private final Pose startPose = new Pose(96, 8, Math.toRadians(90));
    private final Pose Pose2 = new Pose(96, 96, Math.toRadians(45));
    private final Pose Pose3 = new Pose(110, 83.5, Math.toRadians(0));
    private final Pose Pose4 = new Pose(115, 83.5, Math.toRadians(0));
    private final Pose Pose5 = new Pose(120, 83.5, Math.toRadians(0));
    private final Pose Pose6 = new Pose(96, 96, Math.toRadians(45));
    private final Pose Pose7 = new Pose(35, 83.5, Math.toRadians(180));
    private final Pose Pose8 = new Pose(29, 83.5, Math.toRadians(180));
    private final Pose Pose9 = new Pose(24, 83.5, Math.toRadians(180));
    private final Pose Pose10 = new Pose(96, 96, Math.toRadians(45));

    public void stateMachine() {
        switch (pathState) {
            case 0:
                pathState++;
                break;
            case 1:
                follower.followPath(path1);
                pathState++;
                break;
            case 2:
                follower.followPath(path2);
                pathState++;
                break;
            case 3:
                follower.followPath(path3);
                pathState++;
                break;
            case 4:
                follower.followPath(path4);
                pathState++;
                break;
            case 5:
                follower.followPath(path5);
                pathState++;
                break;
            case 6:
                follower.followPath(path6);
                pathState++;
                break;
            case 7:
                follower.followPath(path7);
                pathState++;
                break;
            case 8:
                follower.followPath(path8);
                pathState++;
                break;
            case 9:
                follower.followPath(path9);
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

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            telemetry.addData("FATAL VISION ERROR", "Camera failed to initialize. Check config name: Webcam 1");
            telemetry.addData("STATUS", "Running WITHOUT Camera Detection!");
            telemetry.update();
            visionPortal = null;
        }

        telemetry.addData("Status", "Ready. Waiting for Start.");
        telemetry.addData("path state", pathState);
        telemetry.update();

        waitForStart();

        if (USE_WEBCAM && visionPortal != null) {
            setManualExposure(6, 250);
        }

        if (visionPortal != null) {
            while (opModeIsActive() && !targetFound) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                if (!currentDetections.isEmpty()) {
                    telemetry.addData("# AprilTags Detected", currentDetections.size());
                }

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (detection.id == GPP_TAG_ID) {
                            buildPathsGPP();
                            targetFound = true;
                            foundID = GPP_TAG_ID;
                            break;
                        } else if (detection.id == PGP_TAG_ID) {
                            buildPathsPGP();
                            targetFound = true;
                            foundID = PGP_TAG_ID;
                            break;
                        } else if (detection.id == PPG_TAG_ID) {
                            buildPathsPPG();
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
            buildPathsGPP();
        }

        stateMachine();

        while (opModeIsActive()) {
            follower.update();

            if (pathState > 0 && pathState < 10) {
                if (!follower.isBusy()) {
                    stateMachine();
                }
            } else if (pathState >= 10) {
                stop();
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

    private void buildPathsPGP() {
        path1 = new Path(new BezierLine(startPose, new Pose(startPose.getX(), 50, startPose.getHeading())));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading());

        path2 = new Path(new BezierLine(new Pose(startPose.getX(), 50, startPose.getHeading()), Pose7));
        path2.setLinearHeadingInterpolation(startPose.getHeading(), Pose7.getHeading());

        path3 = null; path4 = null; path5 = null; path6 = null; path7 = null; path8 = null; path9 = null;
    }

    public void buildPathsGPP() {
        path1 = new Path(new BezierLine(startPose, Pose2));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), Pose2.getHeading());

        path2 = new Path(new BezierLine(Pose2, Pose3));
        path2.setLinearHeadingInterpolation(Pose2.getHeading(), Pose3.getHeading());

        path3 = new Path(new BezierLine(Pose3, Pose4));
        path3.setLinearHeadingInterpolation(Pose3.getHeading(), Pose4.getHeading());

        path4 = new Path(new BezierLine(Pose4, Pose5));
        path4.setLinearHeadingInterpolation(Pose4.getHeading(), Pose5.getHeading());

        path5 = new Path(new BezierLine(Pose5, Pose6));
        path5.setLinearHeadingInterpolation(Pose5.getHeading(), Pose6.getHeading());

        path6 = new Path(new BezierLine(Pose6, Pose7));
        path6.setLinearHeadingInterpolation(Pose6.getHeading(), Pose7.getHeading());

        path7 = new Path(new BezierLine(Pose7, Pose8));
        path7.setLinearHeadingInterpolation(Pose7.getHeading(), Pose8.getHeading());

        path8 = new Path(new BezierLine(Pose8, Pose9));
        path8.setLinearHeadingInterpolation(Pose8.getHeading(), Pose9.getHeading());

        path9 = new Path(new BezierLine(Pose9, Pose10));
        path9.setLinearHeadingInterpolation(Pose9.getHeading(), Pose10.getHeading());
    }

    private void buildPathsPPG() {
        path1 = new Path(new BezierLine(startPose, new Pose(startPose.getX(), 72, startPose.getHeading())));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading());

        path2 = new Path(new BezierLine(new Pose(startPose.getX(), 72, startPose.getHeading()), Pose9));
        path2.setLinearHeadingInterpolation(startPose.getHeading(), Pose9.getHeading());

        path3 = null; path4 = null; path5 = null; path6 = null; path7 = null; path8 = null; path9 = null;
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        double timeLimit = 5.0;
        double startTime = getRuntime();

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();

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

            telemetry.addData("Camera", "Exposure Set");
            telemetry.update();
        }
    }
}