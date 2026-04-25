package org.firstinspires.ftc.teamcode.quesobowl.red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.getSpeeds;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;


@Autonomous(name = "red auto")
public class redMotifRailing extends LinearOpMode {
    private Follower follower;
    private int pathState;
    private ElapsedTime actionTimer = new ElapsedTime();

    getSpeeds speeds = new getSpeeds();

    double INTAKE_IN_POWER = 1;

    private final Pose START_POSE = new Pose(20, 117, Math.toRadians(270));
    private final Pose SHOOT_POS = new Pose(96, 96, Math.toRadians(45));

    private final Pose COLLECT_POS_1 = new Pose(90, 87, Math.toRadians(0));
    private final Pose COLLECT_POS_3 = new Pose(119, 87, Math.toRadians(0));

    private final Pose COLLECT_POS_4 = new Pose(90, 64, Math.toRadians(0));
    private final Pose COLLECT_POS_6 = new Pose(115, 64, Math.toRadians(0));

    private final Pose COLLECT_POS_7 = new Pose(90, 38, Math.toRadians(0));
    private final Pose COLLECT_POS_9 = new Pose(119, 38, Math.toRadians(0));
    private final Pose PARK = new Pose(110, 80, Math.toRadians(0));

    private Path smallMove;
    private PathChain collecting1, collecting2, collecting3, backToShot1, backToShot2, backToShot3, Park;
    private DcMotor intake, moveOut;
    private DcMotorEx shooter;

    public void buildPaths() {
        smallMove = new Path(new BezierLine(START_POSE, SHOOT_POS));
        smallMove.setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POS.getHeading());

        collecting1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_1))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_1.getHeading())
                .addPath(new BezierLine(COLLECT_POS_1, COLLECT_POS_3))
                .build();

        backToShot1 = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_POS_3, SHOOT_POS))
                .setLinearHeadingInterpolation(COLLECT_POS_3.getHeading(), SHOOT_POS.getHeading())
                // START SHOOTER WHEN 70% OF THE WAY THERE (ik crazy)
                .addParametricCallback(0.7, () -> shooter.setVelocity(1600))
                .build();

        collecting2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_4))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_4.getHeading())
                .addPath(new BezierLine(COLLECT_POS_4, COLLECT_POS_6))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        backToShot2 = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_POS_6, SHOOT_POS))
                .setLinearHeadingInterpolation(COLLECT_POS_6.getHeading(), SHOOT_POS.getHeading())
                .build();

        collecting3 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_7))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_7.getHeading())
                .addPath(new BezierLine(COLLECT_POS_7, COLLECT_POS_9))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        backToShot3 = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_POS_9, SHOOT_POS))
                .setLinearHeadingInterpolation(COLLECT_POS_9.getHeading(), SHOOT_POS.getHeading())
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, PARK))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), PARK.getHeading())
                .build();
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(START_POSE);
        pathState = 0;

        intake = hardwareMap.get(DcMotor.class, "intake");
        moveOut = hardwareMap.get(DcMotor.class, "moveOut");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        moveOut.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autoPathUpdate();
            telemetry.addData("Path State", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("velocity", shooter.getVelocity());
            telemetry.update();
        }
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.setVelocity(speeds.START_VELOCITY);
                follower.followPath(smallMove, true);
                pathState++;
                sleep(1800);
                break;
            case 1:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState++;
                }
                break;
            case 2:
                shooter.setVelocity(speeds.SHOOTER_VELOCITY);
                moveOut.setPower(speeds.GATE_SPEED);
                if (actionTimer.seconds() > 1) startIntake();
//                if (actionTimer.seconds() > 2)
                if (actionTimer.seconds() > speeds.SHOOTING_TIME) {
                    moveOut.setPower(0);
                    pathState++;
                }
                break;
            case 3:
                startIntake();
                follower.followPath(collecting1, speeds.FIRST_SPEED, true);
                pathState++;
                break;
            case 4:
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;
            case 5:
                follower.followPath(backToShot1, true);
                pathState++;
                break;
            case 6:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState++;
                }
                break;
            case 7:
                shooter.setVelocity(speeds.SHOOTER_VELOCITY);
//                if (actionTimer.seconds() > 2)
                moveOut.setPower(speeds.GATE_SPEED);
                if (actionTimer.seconds() > speeds.SHOOTING_TIME) {
                    moveOut.setPower(0);
                    pathState++;
                }
                break;
            case 8:
                startIntake();
                follower.followPath(collecting2, speeds.SECOND_SPEED, true);
                pathState++;
                break;
            case 9:
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;
            case 10:
                follower.followPath(backToShot2, true);
                pathState++;
                break;
            case 11:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState++;
                }
                break;
            case 12:
                shooter.setVelocity(speeds.SHOOTER_VELOCITY);
//                if (actionTimer.seconds() > 2)
                moveOut.setPower(speeds.GATE_SPEED);
                if (actionTimer.seconds() > speeds.SHOOTING_TIME) {
                    moveOut.setPower(0);
                    pathState++;
                }
                break;
            case 13:
                startIntake();
                follower.followPath(collecting3, speeds.THIRD_SPEED, true);
                pathState++;
                break;
            case 14:
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;
            case 15:
                follower.followPath(backToShot3, true);
                pathState++;
                break;
            case 16:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState++;
                }
                break;
            case 17:
                shooter.setVelocity(speeds.SHOOTER_VELOCITY);
//                if (actionTimer.seconds() > 2)
                moveOut.setPower(speeds.GATE_SPEED);
                if (actionTimer.seconds() > speeds.SHOOTING_TIME) {
                    moveOut.setPower(0);
                    pathState++;
                }
                break;
            case 18:
                follower.followPath(Park, true);
                pathState++;
                break;
            case 19:
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;
            default:
                stopIntake();
                requestOpModeStop();
                shooter.setVelocity(0);
                break;
        }
    }

    public void startIntake() {
        intake.setPower(speeds.INTAKE_IN_POWER);
    }

    public void stopIntake() {
        intake.setPower(0);
    }
}