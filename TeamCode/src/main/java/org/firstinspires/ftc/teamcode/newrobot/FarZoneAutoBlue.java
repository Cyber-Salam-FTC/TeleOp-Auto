package org.firstinspires.ftc.teamcode.newrobot;

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
import org.firstinspires.ftc.teamcode.newrobot.pedropathing.Constants;

@Autonomous(name = "Cyber Salam Auto Far Blue Stilgard")
public class FarZoneAutoBlue extends LinearOpMode {
    private Follower follower;
    private int pathState, nextState;
    private double SHOOTING_TIME = 3;
    private ElapsedTime actionTimer = new ElapsedTime();
    private ElapsedTime Timer30 = new ElapsedTime();

    double INTAKE_IN_POWER = 1;

    private final Pose START_POSE = new Pose(60, 8, Math.toRadians(90));
    private final Pose SHOOT_POS = new Pose(64, 23, Math.toRadians(115));

    //    private final Pose COLLECT_POS_1 = new Pose(90, 96, Math.toRadians(0));
//    private final Pose COLLECT_POS_3 = new Pose(114, 96, Math.toRadians(0));
//
//    private final Pose COLLECT_POS_4 = new Pose(90, 72, Math.toRadians(0));
//    private final Pose COLLECT_POS_6 = new Pose(114, 72, Math.toRadians(0));
//
//    private final Pose COLLECT_POS_7 = new Pose(90, 48, Math.toRadians(0));
//    private final Pose COLLECT_POS_9 = new Pose(114, 48, Math.toRadians(0));
    private final Pose PARK = new Pose(34, 80, Math.toRadians(90));
    private final Pose COLLECT_POS = new Pose(21, 23, Math.toRadians(190));
    private final Pose PRECOLLECT = new Pose(44, 23, Math.toRadians(180));

    private PathChain Path1, Path2, Path3, Return, GoToPark;
    private DcMotor intake1, intake2, intake3;
    private DcMotorEx shooter;

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POS))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POS.getHeading())
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, PRECOLLECT))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), PRECOLLECT.getHeading())
                .addPath(new BezierLine(PRECOLLECT, COLLECT_POS))
                .setLinearHeadingInterpolation(PRECOLLECT.getHeading(), COLLECT_POS.getHeading())
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_POS, SHOOT_POS))
                .setLinearHeadingInterpolation(COLLECT_POS.getHeading(), SHOOT_POS.getHeading())
                .build();

        Return = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_POS, SHOOT_POS))
                .setLinearHeadingInterpolation(COLLECT_POS.getHeading(), SHOOT_POS.getHeading())
                .build();


//        Park = follower.pathBuilder()
//                .addPath(new BezierLine(SHOOT_POS, PARK))
//                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), PARK.getHeading())
//                .build();

        GoToPark = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), PARK))
                .setLinearHeadingInterpolation(follower.getHeading(), PARK.getHeading())
                .build();
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(START_POSE);
        pathState = 0;

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake3 = hardwareMap.get(DcMotor.class, "intake3");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake3.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        Timer30.reset();
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autoPathUpdate();
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.addData("Timer", Timer30.seconds());
            telemetry.update();
        }
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                shoot();
                sleep(2000);
                follower.followPath(Path1, 0.5, true);
                pathState = 99;
                nextState = 1;
                actionTimer.reset();
                break;
            case 1:
                startIntake();
                intake3.setPower(1);
                if (actionTimer.seconds() >= SHOOTING_TIME) pathState = 2;
                break;
            case 2:
                startIntake();
                follower.followPath(Path2);
                pathState = 99;
                nextState = 3;
                break;
            case 3:
                shoot();
                intake3.setPower(0);
                sleep(500);
                follower.followPath(Path3);
                pathState = 99;
                nextState = 1;
                break;

            case 98:
                follower.followPath(GoToPark);
//                pathState = 99;
                if (!follower.isBusy()) {
                    pathState = 50;
                }
                break;
            case 99:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    if (Timer30.seconds() < 25) {
                        pathState = nextState;
                    } else {
                        pathState = 98;
                    }
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
        intake1.setPower(INTAKE_IN_POWER);
        intake2.setPower(INTAKE_IN_POWER);
    }

    public void shoot() {
        shooter.setVelocity(2600);
        intake3.setPower(1);
    }

    public void stopRoller() {
        intake3.setPower(0);
    }

    public void stopIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
    }
}