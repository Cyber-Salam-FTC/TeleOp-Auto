package org.firstinspires.ftc.teamcode.testing.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorMagazine extends OpMode {
    private DcMotor motor;
    boolean leftBumper, rightBumper;
    double currentMotorPosition, newMotorPosition;
    double iterations;
    int delta;
    @Override
    public void init() {
        motor= hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBumper = false;
        rightBumper = false;

        motor.setPower(1);
        iterations = 0;
        delta = 96;


    }
    @Override
    public void loop() {
//        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;
        iterations ++;

        // increment servo position by "delta" such that newServoPosition must be between 0 and 1
//        if (leftBumper && iterations == 1) {
//            if (newMotorPosition >= 0 && newMotorPosition+delta <= 1) {
//                newMotorPosition = newMotorPosition + delta;
//                motor.getCurrentPosition(newMotorPosition);
//            }
//        }
        // decrement servo position by "delta" such that newServoPosition must be between 0 and 1
        if (rightBumper && iterations == 1) {
            motor.setTargetPosition(delta);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            if (newMotorPosition >= 0 && newMotorPosition - delta >= 0) {
//                newMotorPosition = newMotorPosition - delta;
//                motor.getCurrentPosition(newMotorPosition);
//            }
        }
        if (iterations > 50) {
            iterations = 0;
        }
        // if we want to reset servo position ...
//        if (gamepad1.options) {
//            motor.getCurrentPosition(0);
//        }

//        telemetry.addData("currentServoPosition", currentMotorPosition);
//        telemetry.addData("newServoPosition", newMotorPosition);
//        telemetry.addData("leftBumper", gamepad1.left_trigger);
        telemetry.addData("rightBumper", gamepad1.right_trigger);
//        telemetry.addData("options button", gamepad1.options);
        telemetry.addData("iterations", iterations);
        telemetry.update();
    }
}