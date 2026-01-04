package org.firstinspires.ftc.teamcode.newrobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.newrobot.pedropathing.Constants;

@Autonomous(name = "Cyber Salam Auto Red | 26903 ")
public class Auto extends LinearOpMode {
    private Follower follower;
    private int pathState, SHOOTING_TIME;
    private ElapsedTime actionTimer = new ElapsedTime();

    double INTAKE_IN_POWER = 0.8;
    double INTAKE_OUT_POWER = -0.8;

    private final Pose START_POSE = new Pose(123, 123.8, Math.toRadians(38));
    private final Pose SHOOT_POS = new Pose(96, 96, Math.toRadians(38));
    private final Pose EMPTY = new Pose(130, 70, Math.toRadians(0));
    private final Pose MIDPOINT = new Pose(107, 75, Math.toRadians(0));
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
    private DcMotor intake1;
    private DcMotorEx shooter;
    private Limelight3A limelight3A;
    private IMU imu;

    double distance;

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
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(START_POSE);
        pathState = 0;

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        intake1.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SHOOTING_TIME = 4;

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
        switch (pathState) {
            case 0:
                follower.followPath(smallMove, true);
                pathState++;
                break;
            case 1:
                if(!follower.isBusy()){
                    actionTimer.reset();
                    pathState++;
                }
                break;
            case 2:
                shoot();
                if(actionTimer.seconds() > SHOOTING_TIME){
                    shooter.setVelocity(0);
                    pathState++;
                }
                break;
            case 3:
                startIntake(intake1);
                follower.followPath(collecting1, 0.5, true);
                pathState++;
                break;
            case 4:
                if(!follower.isBusy()){
                    pathState++;
                }
                break;
            case 5:
                follower.followPath(backToShot1, true);
                pathState++;
                break;
            case 6:
                if(!follower.isBusy()){
                    actionTimer.reset();
                    pathState++;
                }
                break;
            case 7:
                shoot();
                if(actionTimer.seconds() > SHOOTING_TIME){
                    shooter.setVelocity(0);
                    pathState++;
                }
                break;
            case 8:
                startIntake(intake1);
                follower.followPath(collecting2, 0.5, true);
                pathState++;
                break;
            case 9:
                if(!follower.isBusy()){
                    pathState++;
                }
                break;
            case 10:
                follower.followPath(backToShot2, true);
                pathState++;
                break;
            case 11:
                if(!follower.isBusy()){
                    actionTimer.reset();
                    pathState++;
                }
                break;
            case 12:
                shoot();
                if(actionTimer.seconds() > SHOOTING_TIME){
                    shooter.setVelocity(0);
                    pathState++;
                }
                break;
            default:
                stopIntake(intake1);
                shooter.setVelocity(0);
                break;
        }
    }

    public void shoot() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            distance = getDistanceFromTag(llResult.getTa());
            shooter.setVelocity(getVelocity(distance));
        }
    }

    public void startIntake(DcMotor motor1) {
        motor1.setPower(INTAKE_IN_POWER);
    }

    public void stopIntake(DcMotor motor1) {
        motor1.setPower(0);
    }

    public double getDistanceFromTag(double ta) {
        double scale = 3783.447;
        return Math.sqrt(scale / ta);
    }

    public double getVelocity(double dist) {
        double a = 0.0447472;
        return ((a * (Math.pow(dist, 2))) + (3.81821 * dist) + 1353.07954);
    }
}