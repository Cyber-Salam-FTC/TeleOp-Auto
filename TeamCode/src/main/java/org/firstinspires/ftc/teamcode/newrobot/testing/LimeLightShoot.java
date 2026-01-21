package org.firstinspires.ftc.teamcode.newrobot.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.newrobot.cybersalam.hardware.MecanumDrive;

@TeleOp
public class LimeLightShoot extends OpMode {

    private DcMotorEx shooter;

    int intakeState = 0;
    double INTAKE_IN_POWER = 1;
    double INTAKE_OUT_POWER = -1;
    double PUSH_IN_POWER = 1;


    private DcMotor intake1, intake2, intake3;


    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake3 = hardwareMap.get(DcMotor.class, "intake3");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");


        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake3.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            shooter.setVelocity(1600);
        }

        if (gamepad2.dpad_down) {
            shooter.setVelocity(2800);
        }

        if (gamepad2.dpad_right) {
            shooter.setVelocity(5000);
        }

        MecanumDrive drive = new MecanumDrive();

        drive.init(hardwareMap);

        double forward = gamepad2.right_trigger - gamepad2.left_trigger;
        double strafe = gamepad2.left_stick_x;
        double rotate = gamepad2.right_stick_x;

        drive.drive(forward, strafe, rotate);

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
