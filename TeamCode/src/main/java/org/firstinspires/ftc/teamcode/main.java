package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cybersalam.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@TeleOp(name = "Main TeleOp")
public class main extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor moveOut = hardwareMap.get(DcMotor.class, "moveOut");

        Follower follower = Constants.createFollower(hardwareMap);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        double CLOSE_VELOCITY = 1011;
        double INTAKE_SPEED = 1;
        double GATE_SPEED = 0.85;

        boolean shooting = true;

        MecanumDrive drive = new MecanumDrive();
        drive.init(hardwareMap);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();
            follower.update();
            Pose currentPose = follower.getPose();

            Pose startPose = new Pose(100, 72, Math.toRadians(90));
            follower.setStartingPose(startPose);

            double forward = gamepad1.right_trigger - gamepad1.left_trigger;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            drive.drive(forward, strafe, rotate);
            Pose testPose = new Pose(72, 72);

            double VELOCITY = autoTrackVelocity(currentPose);

            if (gamepad2.dpad_left) {
                shooter.setVelocity(autoTrackVelocity(currentPose));
            }

            if (gamepad2.dpad_right) {
                shooter.setVelocity(1600);
            }

            if (gamepad2.dpad_down) {
                shooter.setVelocity(0);
            }

            if (gamepad2.cross) {
                intake.setPower(INTAKE_SPEED);
            }

            if (gamepad2.circle) {
                intake.setPower(0);
            }

            if (gamepad2.square) {
                moveOut.setPower(GATE_SPEED);
            } else {
                moveOut.setPower(0);
            }

            if (gamepad2.triangle) {
                intake.setPower(-1);
            }

            telemetry.addData("Current velocity (TPS)", shooter.getVelocity());
            telemetry.addData("pos", follower.getPose());
            telemetry.update();
        }
    }

    public double rpmToTps(double rpm) {
        return rpm * 28.0 / 60.0;
    }

    public double autoTrackVelocity(Pose currentPose) {
        double horizontalDist = inchesToMeters(getDistanceFormula(currentPose));
        double heightDiff = inchesToMeters(40 - 15);
        double angle = Math.toRadians(50);
        double g = 9.81;
        double wheelR = mmToM(36);

        if (horizontalDist <= 0) return 0;

        double denominator = Math.sin(2 * angle) - (2 * Math.cos(angle) * Math.cos(angle) * heightDiff / horizontalDist);

        if (denominator <= 0) return 0;

        double initVel = Math.sqrt((g * horizontalDist) / denominator);

        double rpm = getRpm(initVel, wheelR);

        return rpmToTps(rpm);
    }

    public double getDistanceFormula(Pose pose) {
        double x1 = pose.getX();
        double y1 = pose.getY();

        double x2 = 134;
        double y2 = 138;

        double deltaX = x2 - x1;
        double deltaY = y2 - y1;

        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    public double getRpm(double initVel, double wheelR) {
        return (initVel / (2 * Math.PI * wheelR)) * 60;
    }

    public double mmToM(double mm) {
        return mm / 1000;
    }

    public double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}