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
    private int pathState;

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1;
    private double INTAKE_IN_POWER = 1.0;

    private final Pose START_POSE = new Pose(123, 123.8, Math.toRadians(38));
    private final Pose SHOOT_POS = new Pose(121, 121.3, Math.toRadians(38));
    private final Pose EMPTY = new Pose(130, 70, Math.toRadians(0));
    private final Pose MIDPOINT = new Pose(110, 75, Math.toRadians(0));
    private final Pose COLLECT_POS_1 = new Pose(90, 90, Math.toRadians(0));
    private final Pose COLLECT_POS_2 = new Pose(103, 90, Math.toRadians(0));
    private final Pose COLLECT_POS_3 = new Pose(114, 90, Math.toRadians(0));
    private final Pose COLLECT_POS_4 = new Pose(90, 72, Math.toRadians(0));
    private final Pose COLLECT_POS_5 = new Pose(103, 72, Math.toRadians(0));
    private final Pose COLLECT_POS_6 = new Pose(107, 72, Math.toRadians(0));
    private final Pose COLLECT_POS_7 = new Pose(90, 48, Math.toRadians(0));
    private final Pose COLLECT_POS_8 = new Pose(103, 48, Math.toRadians(0));
    private final Pose COLLECT_POS_9 = new Pose(107, 48, Math.toRadians(0));

    private Path smallMove, backToShot1, backToShot2, backToShot3;
    private PathChain collecting1, collecting2, collecting3, empty;
    private DcMotor catapult1, catapult2, intake;

    public void buildPaths() {
        smallMove = new Path(new BezierLine(START_POSE, SHOOT_POS));
        smallMove.setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POS.getHeading());

        collecting1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_1))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_1.getHeading())
                .addPath(new BezierLine(COLLECT_POS_1, COLLECT_POS_2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(COLLECT_POS_2, COLLECT_POS_3))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        empty = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_POS_3, MIDPOINT))
                .setLinearHeadingInterpolation(COLLECT_POS_3.getHeading(), MIDPOINT.getHeading())
                .addPath(new BezierLine(MIDPOINT, EMPTY))
                .setLinearHeadingInterpolation(MIDPOINT.getHeading(), EMPTY.getHeading())
                .build();

        backToShot1 = new Path(new BezierLine(COLLECT_POS_3, SHOOT_POS));
        backToShot1.setLinearHeadingInterpolation(COLLECT_POS_3.getHeading(), SHOOT_POS.getHeading());

        collecting2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_4))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_4.getHeading())
                .addPath(new BezierLine(COLLECT_POS_4, COLLECT_POS_5))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(COLLECT_POS_5, COLLECT_POS_6))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        backToShot2 = new Path(new BezierLine(COLLECT_POS_6, SHOOT_POS));
        backToShot2.setLinearHeadingInterpolation(COLLECT_POS_6.getHeading(), SHOOT_POS.getHeading());

        collecting3 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_7))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_7.getHeading())
                .addPath(new BezierLine(COLLECT_POS_7, COLLECT_POS_8))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(COLLECT_POS_8, COLLECT_POS_9))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        backToShot3 = new Path(new BezierLine(COLLECT_POS_9, SHOOT_POS));
        backToShot3.setLinearHeadingInterpolation(COLLECT_POS_9.getHeading(), SHOOT_POS.getHeading());
    }

    @Override
    public void runOpMode() {
        follower = EveryBotConstants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(START_POSE);
        pathState = 0;

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");

        intake.setDirection(DcMotor.Direction.REVERSE);
        catapult1.setDirection(DcMotor.Direction.FORWARD);
        catapult2.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autoPathUpdate();
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Path State", pathState);
            telemetry.update();
        }
    }

    public void autoPathUpdate() {
        if (follower.isBusy()) return;

        switch (pathState) {
            case 0:
                follower.followPath(smallMove, true);
                pathState++;
                break;
            case 1:
                shoot();
                pathState++;
                break;
            case 2:
                intake.setPower(INTAKE_IN_POWER);
                follower.followPath(collecting1, 0.5, true);
                pathState++;
                break;
            case 3:
                pathState++;
                break;
            case 4:
                follower.followPath(backToShot1, true);
                pathState++;
                break;
            case 5:
                shoot();
                pathState++;
                break;
            case 6:
                intake.setPower(INTAKE_IN_POWER);
                follower.followPath(collecting2, 0.5,true);
                pathState++;
                break;
            case 7:
                follower.followPath(backToShot2, true);
                pathState++;
                break;
            case 8:
                shoot();
                pathState++;
                break;
            case 9:
                intake.setPower(INTAKE_IN_POWER);
                follower.followPath(collecting3, 0.5,true);
                pathState++;
                break;
            case 10:
                follower.followPath(backToShot3, true);
                pathState++;
                break;
            case 11:
                shoot();
                pathState++;
                break;
            default:
                requestOpModeStop();
                break;
        }
    }

    public void shoot() {
        catapultUp();
        sleep(1500);
        catapultDown();
    }

    public void catapultUp() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
    }

    public void catapultDown() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
    }
}