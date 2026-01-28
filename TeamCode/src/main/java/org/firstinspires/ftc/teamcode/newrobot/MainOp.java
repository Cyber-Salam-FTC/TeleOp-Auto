package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.newrobot.cybersalam.hardware.MecanumDrive;

@TeleOp(name = "Cyber Salam - Stilgar TeleOp")
public class MainOp extends LinearOpMode {
    double forward, strafe, rotate, distance;
    double nextAlertTime = 0;
    double nextThirtySecAlert = 30.0;
    boolean isEndGame = false;
    int intakeState = 0;
    double INTAKE_IN_POWER = 1;
    double INTAKE_OUT_POWER = -1;
    double PUSH_IN_POWER = 1;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive();
        drive.init(hardwareMap);

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor intake1 = hardwareMap.get(DcMotor.class, "intake1");
        DcMotor intake2 = hardwareMap.get(DcMotor.class, "intake2");
        DcMotor intake3 = hardwareMap.get(DcMotor.class, "intake3");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        limelight3A.pipelineSwitch(0);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake3.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight3A.start();

        waitForStart();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double endGameThreshold = 90.0;
            double matchEnd = 120.0;

            if (currentTime >= nextThirtySecAlert && currentTime < endGameThreshold) {
                gamepad1.rumble(300);
                gamepad2.rumble(300);
                nextThirtySecAlert += 30.0;
            }

            if (currentTime >= endGameThreshold && currentTime < 115.0) {
                if (currentTime >= nextAlertTime) {
                    gamepad1.rumbleBlips(3);
                    gamepad2.rumbleBlips(3);
                    nextAlertTime = currentTime + 15.0;
                    isEndGame = true;
                }
            } else if (currentTime >= 115.0 && currentTime <= matchEnd) {
                double remainingTime = matchEnd - currentTime;
                double rumbleInterval = Math.max(0.1, remainingTime / 5.0);
                if (currentTime >= nextAlertTime) {
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                    nextAlertTime = currentTime + rumbleInterval;
                }
            }

            forward = gamepad1.right_trigger - gamepad1.left_trigger;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
            drive.drive(forward, strafe, rotate);

            if (gamepad2.circle) {
                intakeState = 0;
            } else if (gamepad2.cross) {
                intakeState = 1;
            } else if (gamepad2.triangle) {
                intakeState = -1;
            }

            if (intakeState == 1) {
                startMainIntake(intake1, intake2);
            } else if (intakeState == -1) {
                intakeMainOut(intake1, intake2);
            } else {
                stopMainIntake(intake1, intake2);
            }

            if (gamepad2.square) {
                intake3.setPower(PUSH_IN_POWER);
            } else {
                intake3.setPower(0);
            }

            if (gamepad2.dpad_up) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    distance = getDistanceFromTag(llResult.getTa());
                    shooter.setVelocity(getVelocity(distance));
                    telemetry.addData("Distance", distance);
                    telemetry.addData("Outtake Velocity", shooter.getVelocity());
                }
            }

            if (gamepad2.dpad_right) {
                shooter.setVelocity(1600);
            }

            if (gamepad2.dpad_down) {
                shooter.setVelocity(0);
            }

            telemetry.update();
        }
    }

    public double getDistanceFromTag(double ta) {
        double scale = 72.06169;
        double power = -0.509117;
        double distance = scale * Math.pow(ta, power);
        return distance;
    }

    public double getVelocity(double dist) {
        double a = 0.29714;
        return ((a*Math.pow(dist, 2)) + (20.07681*dist) + 0);
    }

    public void startMainIntake(DcMotor motor1, DcMotor motor2) {
        motor1.setPower(INTAKE_IN_POWER);
        motor2.setPower(INTAKE_IN_POWER);
    }

    public void stopMainIntake(DcMotor motor1, DcMotor motor2) {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public void intakeMainOut(DcMotor motor1, DcMotor motor2) {
        motor1.setPower(INTAKE_OUT_POWER);
        motor2.setPower(INTAKE_OUT_POWER);
    }
}