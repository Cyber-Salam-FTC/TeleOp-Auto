package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedropathing.EveryBotConstants;


@Autonomous(name = "Cyber Salam EveryBot Auto Red| 26903 ")
public class EveryBotAutoRed extends LinearOpMode {
    private Follower follower;

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1;
    private double CATAPULT_HOLD_POWER = 0.2;

    private double INTAKE_IN_POWER = 1.0;
    private double INTAKE_OUT_POWER = -0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    private int pathState;

    //    start pose
    private final Pose START_POSE = new Pose(123, 123.8, Math.toRadians(38));

    //    shot pos
    private final Pose SHOOT_POS = new Pose(121, 121.3, Math.toRadians(38));

//    empty bucket

    private final Pose EMPTY = new Pose(130, 70, Math.toRadians(0));


    //  collect positions
    private final Pose COLLECT_POS_1 = new Pose(90, 90, Math.toRadians(0));
    private final Pose COLLECT_POS_2 = new Pose(103, 90, Math.toRadians(0));
    private final Pose COLLECT_POS_3 = new Pose(107, 90, Math.toRadians(0));
    private final Pose COLLECT_POS_4 = new Pose(90, 72, Math.toRadians(0));
    private final Pose COLLECT_POS_5 = new Pose(103, 72, Math.toRadians(0));
    private final Pose COLLECT_POS_6 = new Pose(107, 72, Math.toRadians(0));
    private final Pose COLLECT_POS_7 = new Pose(90, 48, Math.toRadians(0));
    private final Pose COLLECT_POS_8 = new Pose(103, 48, Math.toRadians(0));
    private final Pose COLLECT_POS_9 = new Pose(107, 48, Math.toRadians(0));

    private Path smallMove, empty, backToShot1, backToShot2, backToShot3, backToShot4, last;
    private PathChain collecting1, collecting2, collecting3;

    private DcMotor catapult1, catapult2, intake;

    public void buildPaths() {
//        from start to shot
        smallMove = new Path(new BezierLine(START_POSE, SHOOT_POS));
        smallMove.setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POS.getHeading());
//        shot to collect
        collecting1 = follower.pathBuilder()
                .setVelocityConstraint(12)
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_1))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(COLLECT_POS_1, COLLECT_POS_2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(COLLECT_POS_2, COLLECT_POS_3))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


//        empty goal

        empty = new Path(new BezierLine(COLLECT_POS_3, EMPTY));
        empty.setVelocityConstraint(52);
        empty.setLinearHeadingInterpolation(COLLECT_POS_3.getHeading(), EMPTY.getHeading());


        backToShot1 = new Path(new BezierLine(EMPTY, SHOOT_POS));
        backToShot1.setVelocityConstraint(52);
        backToShot1.setLinearHeadingInterpolation(EMPTY.getHeading(), SHOOT_POS.getHeading());

        collecting2 = follower.pathBuilder()
                .setVelocityConstraint(12)
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_4))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_4.getHeading())
                .addPath(new BezierLine(COLLECT_POS_4, COLLECT_POS_5))
                .setLinearHeadingInterpolation(COLLECT_POS_4.getHeading(), COLLECT_POS_5.getHeading())
                .addPath(new BezierLine(COLLECT_POS_5, COLLECT_POS_6))
                .setLinearHeadingInterpolation(COLLECT_POS_5.getHeading(), COLLECT_POS_6.getHeading())
                .build();

        backToShot2 = new Path(new BezierLine(COLLECT_POS_6, SHOOT_POS));
        backToShot2.setVelocityConstraint(52);
        backToShot2.setLinearHeadingInterpolation(COLLECT_POS_6.getHeading(), SHOOT_POS.getHeading());

        collecting3 = follower.pathBuilder()
                .setVelocityConstraint(12)
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_7))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_7.getHeading())
                .addPath(new BezierLine(COLLECT_POS_7, COLLECT_POS_8))
                .setLinearHeadingInterpolation(COLLECT_POS_7.getHeading(), COLLECT_POS_8.getHeading())
                .addPath(new BezierLine(COLLECT_POS_8, COLLECT_POS_9))
                .setLinearHeadingInterpolation(COLLECT_POS_8.getHeading(), COLLECT_POS_9.getHeading())
                .build();

        backToShot3 = new Path(new BezierLine(COLLECT_POS_9, SHOOT_POS));
        backToShot3.setVelocityConstraint(52);
        backToShot3.setLinearHeadingInterpolation(COLLECT_POS_9.getHeading(), SHOOT_POS.getHeading());

        last = new Path(new BezierLine(SHOOT_POS, COLLECT_POS_1));
        last.setVelocityConstraint(52);
        last.setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_1.getHeading());


    }


    @Override
    public void runOpMode() {

        follower = EveryBotConstants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(START_POSE);
        setPathState(0);

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");

        intake.setDirection(DcMotor.Direction.REVERSE);
        catapult1.setDirection(DcMotor.Direction.FORWARD);
        catapult2.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            catapultDown();
            intake.setPower(INTAKE_IN_POWER);

            autoPathUpdate();
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();

                telemetry.addData("path state", pathState);
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();
            }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(smallMove);
//                sleep(16000);
//                shoot();
                pathState++;
                break;
            case 1:
                shoot();
                pathState++;
                break;
            case 2:
                follower.followPath(collecting1);
                pathState++;
                break;
            case 3:
                follower.followPath(empty);
                pathState++;
                break;
            case 4:
                follower.followPath(backToShot1);
                pathState++;
                break;
            case 5:
                shoot();
                pathState++;
                break;
            case 6:
                follower.followPath(collecting2);
                pathState++;
                break;
            case 7:
                follower.followPath(backToShot2);
                pathState++;
                break;
            case 8:
                shoot();
                pathState++;
                break;
            case 9:
                follower.followPath(collecting3);
                pathState++;
                break;
            case 10:
                follower.followPath(backToShot3);
                pathState++;
                break;
            case 11:
                shoot();
                pathState++;
                break;
            default:
                requestOpModeStop();
        }
    }

    public void catapultUp() {
        catapult1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
    }

    public void catapultDown() {
        catapult1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
    }

    public void shoot() {
        catapultUp();
        sleep(1500);
        catapultDown();
    }
}
