package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "AdeelRotationTest")

public class AdeelRotationTest extends OpMode {

    public static Follower follower;
    private final Pose startPose = new Pose (0, 0, 0);
    private final Pose endPose = new Pose (24, 24, Math.PI);
    private PathChain path;
/*
    public static double DISTANCE = 40;
    private boolean forward = true;
    private Path forwards;
    private Path backwards;
*/


    @Override
    public void init() {

        DcMotor leftFront, leftRear, rightFront, rightRear;
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        follower = Constants.createFollower(hardwareMap);
        follower.deactivateAllPIDFs();
        follower.activateHeading();
//        buildPaths(); // Initialize paths, including fullPath
//        follower.setStartingPose(startPose);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
//        telemetry.debug("This will activate the heading PIDF(s).");
//        telemetry.debug("The robot will try to stay at a constant heading while you try to turn it.");
//        telemetry.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).");
        telemetry.update();
        follower.update();
//            drawCurrent();
    }

    @Override
    public void start() {

        //        follower.deactivateAllPIDFs();
        follower.activateHeading();

        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), 1)
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading(), 1)
                .build();

//        backwards = new Path(new BezierLine(endPose, startPose));
//        backwards.setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading());

//        path = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .addPath(new BezierLine(pickup1Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();

        follower.followPath(path);


    }



    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
//        drawCurrentAndHistory();

        if (follower.atParametricEnd()) {
            follower.followPath(path);
        }
        telemetry.update();
    }
}

