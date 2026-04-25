package org.firstinspires.ftc.teamcode.quesobowl.red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.getSpeeds;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous
public class redBottomRight extends LinearOpMode {
    public Follower follower;
    public getSpeeds speeds = new getSpeeds();
    private ElapsedTime actionTimer = new ElapsedTime();

    long SHOOTING_TIME_MS;
    boolean pathStarted = false;
    double intakeValue = 0;

    public DcMotor leftFront, leftRear, rightFront, rightRear, intake, gate;
    public DcMotorEx shooter;

    enum States {
        GO_TO_SHOOT,
        SHOOTING,
        FIRST_INTAKE,
        FIRST_COLLECT,
        SECOND_INTAKE,
        SECOND_COLLECT,
        THIRD_INTAKE,
        THIRD_COLLECT,
        PARK,
        END
    }

    States state = States.GO_TO_SHOOT;

    Pose startPose = new Pose(129, 8, Math.toRadians(0));
    Pose scorePose = new Pose(83, 32, Math.toRadians(270));
    Pose intake1HeadingPose = new Pose(88, 33, Math.toRadians(0));
    Pose intake1DeepPose = new Pose(124, 33, Math.toRadians(0));
    Pose intake2HeadingPose = new Pose(88, 57, Math.toRadians(0));
    Pose intake2DeepPose = new Pose(124, 57, Math.toRadians(0));
    Pose intake3HeadingPose = new Pose(88, 81, Math.toRadians(0));
    Pose intake3DeepPose = new Pose(124, 81, Math.toRadians(0));
    Pose parkPose = new Pose(100, 52);

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(DcMotor.class, "moveOut");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SHOOTING_TIME_MS = (long) (speeds.SHOOTING_TIME * 1000);
        follower.setStartingPose(startPose);

        waitForStart();
        actionTimer.reset();

        while (opModeIsActive()) {
            shooter.setVelocity(speeds.SHOOTER_VELOCITY);
            intake.setPower(speeds.INTAKE_IN_POWER);
            follower.update();
            runStateMachine();
            telemetry.addData("State", state);
            telemetry.update();
        }
    }

    public void runStateMachine() {
        switch (state) {
            case GO_TO_SHOOT:
                if (!pathStarted) {
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), scorePose))
                            .setLinearHeadingInterpolation(follower.getHeading(), scorePose.getHeading())
                            .build());
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    actionTimer.reset();
                    state = States.SHOOTING;
                }
                break;

            case SHOOTING:
                gate.setPower(1);
                if (actionTimer.milliseconds() < 500) {
                    shooter.setVelocity(speeds.START_VELOCITY);
                } else {
                    shooter.setVelocity(speeds.SHOOTER_VELOCITY);
                }

                if (actionTimer.milliseconds() > SHOOTING_TIME_MS) {
                    gate.setPower(0);
                    pathStarted = false;
                    if (intakeValue == 0) state = States.FIRST_INTAKE;
                    else if (intakeValue == 1) state = States.SECOND_INTAKE;
                    else if (intakeValue == 2) state = States.THIRD_INTAKE;
                    else state = States.PARK;
                }
                break;

            case FIRST_INTAKE:
                if (!pathStarted) {
                    intake.setPower(speeds.INTAKE_IN_POWER);
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1HeadingPose))
                            .setLinearHeadingInterpolation(follower.getHeading(), intake1HeadingPose.getHeading())
                            .build());
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = States.FIRST_COLLECT;
                }
                break;

            case FIRST_COLLECT:
                if (!pathStarted) {
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1DeepPose))
                            .setConstantHeadingInterpolation(intake1HeadingPose.getHeading())
                            .build(), speeds.FIRST_SPEED, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    intakeValue = 1;
                    state = States.GO_TO_SHOOT;
                }
                break;

            case SECOND_INTAKE:
                if (!pathStarted) {
                    intake.setPower(speeds.INTAKE_IN_POWER);
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake2HeadingPose))
                            .setLinearHeadingInterpolation(follower.getHeading(), intake2HeadingPose.getHeading())
                            .build());
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = States.SECOND_COLLECT;
                }
                break;

            case SECOND_COLLECT:
                if (!pathStarted) {
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake2DeepPose))
                            .setConstantHeadingInterpolation(intake2HeadingPose.getHeading())
                            .build(), speeds.FIRST_SPEED, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    intakeValue = 2;
                    state = States.GO_TO_SHOOT;
                }
                break;

            case THIRD_INTAKE:
                if (!pathStarted) {
                    intake.setPower(speeds.INTAKE_IN_POWER);
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake3HeadingPose))
                            .setLinearHeadingInterpolation(follower.getHeading(), intake3HeadingPose.getHeading())
                            .build());
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = States.THIRD_COLLECT;
                }
                break;

            case THIRD_COLLECT:
                if (!pathStarted) {
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake3DeepPose))
                            .setConstantHeadingInterpolation(intake3HeadingPose.getHeading())
                            .build(), speeds.FIRST_SPEED, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    intakeValue = 3;
                    state = States.GO_TO_SHOOT;
                }
                break;

            case PARK:
                if (!pathStarted) {
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), parkPose))
                            .setLinearHeadingInterpolation(follower.getHeading(), parkPose.getHeading())
                            .build());
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    state = States.END;
                }
                break;

            case END:
                shooter.setVelocity(0);
                intake.setPower(0);
                requestOpModeStop();
                break;
        }
    }
}