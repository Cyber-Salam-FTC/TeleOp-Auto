package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.newrobot.cybersalam.hardware.MecanumDrive;

public class MainOp extends LinearOpMode {

    double forward, strafe, rotate, distance;
    int intakeState = 0;
    double INTAKE_IN_POWER = 0.8;
    double INTAKE_OUT_POWER = -0.8;

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
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        limelight3A.pipelineSwitch(0);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            limelight3A.start();

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
                startIntake(intake1, intake2);
            } else if (intakeState == -1) {
                intakeOut(intake1, intake2);
            } else {
                stopIntake(intake1, intake2);
            }

            if (gamepad2.dpad_up) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    Pose3D botpose = llResult.getBotpose_MT2();
                    distance = getDistanceFromTag(llResult.getTa());
                    shooter.setVelocity(getVelocity(distance));

                    telemetry.addData("Distance", distance);
                    telemetry.addData("Target X", llResult.getTx());
                    telemetry.addData("Outtake Velocity", shooter.getVelocity());
                }
            }

            if (gamepad2.dpad_down) {
                shooter.setVelocity(0);
            }

            telemetry.update();
        }
    }

    public double getDistanceFromTag(double ta) {
        double scale = 3783.447;
        return Math.sqrt(scale / ta);
    }

    public double getVelocity(double dist) {
        double a = 0.0447472;
        return ((a * (Math.pow(dist, 2))) + (3.81821 * dist) + 1353.07954);
    }

    public void startIntake(DcMotor motor1, DcMotor motor2) {
        motor1.setPower(INTAKE_IN_POWER);
        motor2.setPower(INTAKE_IN_POWER);
    }

    public void stopIntake(DcMotor motor1, DcMotor motor2) {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public void intakeOut(DcMotor motor1, DcMotor motor2) {
        motor1.setPower(INTAKE_OUT_POWER);
        motor2.setPower(INTAKE_OUT_POWER);
    }
}