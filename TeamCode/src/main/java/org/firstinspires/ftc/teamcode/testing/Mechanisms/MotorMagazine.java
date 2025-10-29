package org.firstinspires.ftc.teamcode.testing.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Motor Magazine Test")
public class MotorMagazine extends OpMode {
    private DcMotor motor;
    boolean leftBumper, rightBumper;
    double currentMotorPosition, newMotorPosition;
    double iterations;
    int delta;
    @Override
    public void init() {
        motor= hardwareMap.get(DcMotor.class, "rotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBumper = false;
        rightBumper = false;

        motor.setPower(0.2);
        iterations = 0;
        delta = 96;


    }
    @Override
    public void loop() {
        rightBumper = gamepad1.right_bumper;
        iterations = 1;

        if (rightBumper) {
            motor.setTargetPosition(delta);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(0.2);
        }
        if (iterations > 50) {
            iterations = 0;
        }
        telemetry.addData("rightBumper", gamepad1.right_bumper);
        telemetry.addData("currentMotorPosition",currentMotorPosition);
        telemetry.addData("iterations", iterations);
        telemetry.update();
    }
}