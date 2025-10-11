package org.firstinspires.ftc.teamcode.testing.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Servo Test")
public class ServoExamples extends OpMode {
    org.firstinspires.ftc.teamcode.testing.Mechanisms.servo servo = new org.firstinspires.ftc.teamcode.testing.Mechanisms.servo();
    boolean leftBumper, rightBumper;
    double currentServoPosition, newServoPosition;
    double iterations;
    double delta;

    @Override
    public void init() {
        // Initialize servos
        servo.init(hardwareMap);
        leftBumper = false;
        rightBumper = false;

        servo.setServoPos(0);
        newServoPosition = 0;
        iterations = 0;
        delta = 0.2;

    }
    @Override
    public void loop() {
        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;
        iterations ++;

        // increment servo position by "delta" such that newServoPosition must be between 0 and 1
        if (leftBumper && iterations == 1) {
            if (newServoPosition >= 0 && newServoPosition+delta <= 1) {
                newServoPosition = newServoPosition + delta;
                servo.setServoPos(newServoPosition);
            }
        }
        // decrement servo position by "delta" such that newServoPosition must be between 0 and 1
        if (rightBumper && iterations == 1) {
            if (newServoPosition >= 0 && newServoPosition - delta >= 0) {
                newServoPosition = newServoPosition - delta;
                servo.setServoPos(newServoPosition);
            }
        }
        if (iterations > 50) {
            iterations = 0;
        }
        // if we want to reset servo position ...
        if (gamepad1.options) {
            servo.setServoPos(0);
        }

        telemetry.addData("currentServoPosition", currentServoPosition);
        telemetry.addData("newServoPosition", newServoPosition);
        telemetry.addData("leftBumper", gamepad1.left_trigger);
        telemetry.addData("rightBumper", gamepad1.right_trigger);
        telemetry.addData("options button", gamepad1.options);
        telemetry.addData("iterations", iterations);
        telemetry.update();
    }
}
/*
1. set CR to reverse direction
2. set opmode so left trigger sets position of servo and right trigger makes 0 off and 1 fully on
 */