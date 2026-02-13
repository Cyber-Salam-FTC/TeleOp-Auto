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

@Autonomous(name = "Cyber Salam Auto Blue Stilgard")
public class AutoBlue extends LinearOpMode {
    private Follower follower;
    private int pathState;
    private double SHOOTING_TIME = 3;
    private ElapsedTime actionTimer = new ElapsedTime();

    double INTAKE_IN_POWER = 1;

    private final Pose START_POSE = new Pose(21, 123.8, Math.toRadians(142));
    private final Pose SHOOT_POS = new Pose(48, 96, Math.toRadians(135));

    private final Pose COLLECT_POS_1 = new Pose(54, 90, Math.toRadians(180));
    private final Pose COLLECT_POS_3 = new Pose(25, 90, Math.toRadians(180));

    private final Pose COLLECT_POS_4 = new Pose(54, 64, Math.toRadians(180));
    private final Pose COLLECT_POS_6 = new Pose(25, 64, Math.toRadians(180));

    private final Pose COLLECT_POS_7 = new Pose(54, 42, Math.toRadians(180));
    private final Pose COLLECT_POS_9 = new Pose(20, 42, Math.toRadians(180));
    private final Pose PARK = new Pose(23, 96, Math.toRadians(180));

    private Path smallMove;
    private PathChain collecting1, collecting2, collecting3, backToShot1, backToShot2, backToShot3, Park;
    private DcMotor intake1, intake2, intake3;
    private DcMotorEx shooter;

    public void buildPaths() {
        smallMove = new Path(new BezierLine(START_POSE, SHOOT_POS));
        smallMove.setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POS.getHeading());

        collecting1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_1))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_1.getHeading())
                .addPath(new BezierLine(COLLECT_POS_1, COLLECT_POS_3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        backToShot2 = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_POS_6, SHOOT_POS))
                .setLinearHeadingInterpolation(COLLECT_POS_6.getHeading(), SHOOT_POS.getHeading())
                .build();

        collecting3 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POS, COLLECT_POS_7))
                .setLinearHeadingInterpolation(SHOOT_POS.getHeading(), COLLECT_POS_7.getHeading())
                .addPath(new BezierLine(COLLECT_POS_7, COLLECT_POS_9))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autoPathUpdate();
            telemetry.addData("Path State", pathState);
            telemetry.update();
        }
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(smallMove, true);
                pathState++;
                break;
            case 1:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState++;
                }
                break;
            case 2:
                shooter.setVelocity(1600);
                intake3.setPower(1);
                if (actionTimer.seconds() > 1) startIntake();
//                if (actionTimer.seconds() > 2)
                if (actionTimer.seconds() > SHOOTING_TIME) {
                    intake3.setPower(0);
                    pathState++;
                }
                break;
            case 3:
                startIntake();
                follower.followPath(collecting1, 0.75, true);
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
                shooter.setVelocity(1600);
//                if (actionTimer.seconds() > 2)
                intake3.setPower(1);
                if (actionTimer.seconds() > SHOOTING_TIME) {
                    intake3.setPower(0);
                    pathState++;
                }
                break;
            case 8:
                startIntake();
                follower.followPath(collecting2, 0.65, true);
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
                shooter.setVelocity(1600);
//                if (actionTimer.seconds() > 2)
                intake3.setPower(1);
                if (actionTimer.seconds() > SHOOTING_TIME) {
                    intake3.setPower(0);
                    pathState++;
                }
                break;
            case 13:
                startIntake();
                follower.followPath(collecting3, 0.75, true);
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
                shooter.setVelocity(1600);
//                if (actionTimer.seconds() > 2)
                intake3.setPower(1);
                if (actionTimer.seconds() > SHOOTING_TIME) {
                    intake3.setPower(0);
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
        intake1.setPower(INTAKE_IN_POWER);
        intake2.setPower(INTAKE_IN_POWER);
    }

    public void stopIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
    }
}