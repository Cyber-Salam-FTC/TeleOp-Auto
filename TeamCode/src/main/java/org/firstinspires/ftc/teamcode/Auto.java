package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "Auto - Cyber Salam FTC | 26903")
public class Auto extends OpMode {
    private Follower follower;
    private int pathState = 0;
    private Path path1, path2, path3, path4, path5, path6, path7, path8, path9;
    private PathChain fullPath;

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private final Pose startPose = new Pose(96, 96, Math.toRadians(90));
    private final Pose Pose2 = new Pose(96, 120, Math.toRadians(0));
    private final Pose Pose3 = new Pose(110, 83.5, Math.toRadians(0));
    private final Pose Pose4 = new Pose(115, 83.5, Math.toRadians(0));
    private final Pose Pose5 = new Pose(120, 83.5, Math.toRadians(0));
    private final Pose Pose6 = new Pose(96, 96, Math.toRadians(45));
    private final Pose Pose7 = new Pose(35, 83.5, Math.toRadians(180));
    private final Pose Pose8 = new Pose(29, 83.5, Math.toRadians(180));
    private final Pose Pose9 = new Pose(24, 83.5, Math.toRadians(180));
    private final Pose Pose10 = new Pose(96, 96, Math.toRadians(45));

//    private final Pose PoseA = new Pose(0,0,Math.toRadians(90));
//    private final Pose PoseB = new Pose(0,0,Math.toRadians(0));

    public void buildPaths() {
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

    public void stateMachine() {
        if (!follower.isBusy()) {
            switch (pathState) {
                case 0:
                    pathState++;
                    break;
                case 1:
                    follower.followPath(path1);
                    pathState++;
                    requestOpModeStop();
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
                    requestOpModeStop();
                    break;
            }
        }
    }


    @Override
    public void loop() {
        follower.update();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        stateMachine();
    }

    @Override
    public void init() {}
    public void start() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();


        follower.followPath(fullPath);
    }

    @Override
    public void init_loop() {}

//    @Override
//    public void start() {}

    @Override
    public void stop() {}
}