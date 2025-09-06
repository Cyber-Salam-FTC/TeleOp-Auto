package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.pedropathing.FConstants;
import org.firstinspires.ftc.teamcode.pedropathing.LConstants;

@Autonomous(name = "Auto Testing Cyber Salam FTC")
public class AutoTesting extends OpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private boolean pathCompleted = false;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(270));
    private final Pose endPose = new Pose(0, 48, Math.toRadians(270));
    private Path moveForward;

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        moveForward = new Path(new BezierLine(new Point(startPose), new Point(endPose)));
        moveForward.setConstantHeadingInterpolation(startPose.getHeading());

        pathState = 0;
    }

    @Override
    public void loop() {
        if (!pathCompleted) {
            follower.update();
        }

        autoPathUpdate();

        Pose currentPose = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Current Pose", currentPose.toString());
        telemetry.addData("Distance to Target", getDistanceToTarget(currentPose, endPose));
        telemetry.addData("Heading Error (deg)", Math.toDegrees(getHeadingError(currentPose, endPose)));
        telemetry.addData("LF Power", leftFront.getPower());
        telemetry.addData("LR Power", leftRear.getPower());
        telemetry.addData("RF Power", rightFront.getPower());
        telemetry.addData("RR Power", rightRear.getPower());
        telemetry.update();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(moveForward);
                setPathState(1);
                break;

            case 1:
                Pose currentPose = follower.getPose();
                if (hasReachedTarget(currentPose, endPose, 4.0, Math.toRadians(15))) {
                    pathCompleted = true;
                    stopMotors();
                    setPathState(2);
                }
                break;

            case 2:
                // Path complete
                break;
        }
    }

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public boolean hasReachedTarget(Pose current, Pose target, double positionTolerance, double headingTolerance) {
        return getDistanceToTarget(current, target) < positionTolerance &&
                getHeadingError(current, target) < headingTolerance;
    }

    public double getDistanceToTarget(Pose current, Pose target) {
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        return Math.hypot(dx, dy);
    }

    public double getHeadingError(Pose current, Pose target) {
        double diff = Math.abs(current.getHeading() - target.getHeading());
        return (diff > Math.PI) ? (2 * Math.PI - diff) : diff;
    }

    public void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    @Override
    public void stop() {
        stopMotors();
    }
}
