package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "Auto - Cyber Salam FTC | 26903")
public class Auto extends OpMode {
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(72,0,Math.toRadians(90));
    private final Pose endPose = new Pose(72,3,Math.toRadians(90));
    private Path move;

    private DcMotor leftFront, leftRear, rightFront, rightRear;


    public void buildPaths() {
        move = new Path(new BezierLine(startPose, endPose));
        move.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        follower.followPath(move);
    }


    @Override
    public void loop() {
        follower.update();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        if (!follower.isBusy()) {
            pathState = 1;
            requestOpModeStop();
        }
    }

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {

    }

    @Override
    public void stop() {}
}