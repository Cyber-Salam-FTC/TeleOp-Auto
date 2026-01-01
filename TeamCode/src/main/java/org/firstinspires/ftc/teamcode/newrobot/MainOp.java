package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.newrobot.cybersalam.hardware.MecanumDrive;
public class MainOp extends LinearOpMode {

    double forward, strafe, rotate, distance;

    double INTAKE_IN_POWER, INTAKE_OUT_POWER;

    boolean startShootButton, endShootButton, intakeInButton, intakeOutButton;

    @Override
    public void runOpMode() {
//        init motors/mecanum drive
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

        startShootButton = gamepad2.square;
        endShootButton = gamepad2.triangle;

        intakeInButton = gamepad2.left_stick_y > 0;
        intakeOutButton = false;

        if (gamepad2.left_stick_y < 0) {
            intakeInButton = false;
            intakeOutButton = true;
        } else {
            intakeInButton = false;
        }


        waitForStart();

        while (opModeIsActive()) {
            limelight3A.start();

            forward = gamepad1.right_trigger - gamepad1.left_trigger;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            drive.drive(forward, strafe, rotate);


        }

        if (startShootButton) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botpose = llResult.getBotpose_MT2();
                distance = getDistanceFromTag(llResult.getTa());
                telemetry.addData("Distance", distance);
                telemetry.addData("Target X", llResult.getTx());
                telemetry.addData("Target Area", llResult.getTa());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("Outtake Velocity", shooter.getVelocity());
            }

            assert llResult != null;
            shooter.setVelocity(getVelocity(getDistanceFromTag(llResult.getTa())));
        }

        if (endShootButton) {
            shooter.setVelocity(0);
        }

        if (intakeInButton) {
            intake1.setPower(INTAKE_IN_POWER);
            intake2.setPower(INTAKE_IN_POWER);
        }

        if (intakeOutButton) {
            intake1.setPower(INTAKE_OUT_POWER);
            intake2.setPower(INTAKE_OUT_POWER);
        }


    }

    public double getDistanceFromTag(double ta) {
        double scale = 3783.447;
        return Math.sqrt(scale / ta);
    }

    public double getVelocity(double dist) {
        double a = 0.0447472;
        return ((a*(Math.pow(dist, 2))) + (3.81821*dist) + 1353.07954);
    }
}
