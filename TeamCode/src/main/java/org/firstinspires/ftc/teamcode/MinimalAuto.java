package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "Minimal Auto - Cyber Salam FTC | 26903")
public class MinimalAuto extends OpMode {
    private Follower follower;
    private int pathState = 0;
    private Path path1, path2, path3, path4, path5, path6, path7, path8, path9;

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private final Pose startPose = new Pose(96, 8, Math.toRadians(90));
    private final Pose Pose2 = new Pose(96, 34, Math.toRadians(90));

    public void buildPaths() {
        path1 = new Path(new BezierLine(startPose, Pose2));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), Pose2.getHeading());
    }

    public void stateMachine() {
        switch (pathState) {
            case 0:
                pathState++;
                break;
            case 1:
                follower.followPath(path1);
                pathState++;
                break;
            default:
                requestOpModeStop();
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();

        if (pathState > 0 && pathState < 3) {
            if (!follower.isBusy()) {
                stateMachine();
            }
        } else if (pathState >= 3) {
            requestOpModeStop();
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init() {
        buildPaths();
    }

    public void start() {
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

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        stateMachine();
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {}
}