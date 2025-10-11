package org.firstinspires.ftc.teamcode.testing.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Servo Test")
public class ServoExamples extends OpMode {
    org.firstinspires.ftc.teamcode.testing.Mechanisms.servo servo = new org.firstinspires.ftc.teamcode.testing.Mechanisms.servo();
    double leftTrigger, rightTrigger;
    double currentServoPosition, newServoPosition;
    double iterations;

    @Override
    public void init() {
        // Initialize servos
        servo.init(hardwareMap);
//        leftTrigger = 0.0;
//        rightTrigger = 0.0;

        servo.setServoPos(0);
        newServoPosition = 0;
        iterations = 0;

    }
    @Override
    public void loop() {
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;
        iterations ++;

        if (leftTrigger > 0 && iterations == 1) {
            if (newServoPosition > 0 && newServoPosition+0.3 < 1) {
                newServoPosition = newServoPosition + 0.3;
                servo.setServoPos(newServoPosition);
            }
        }
        if (rightTrigger > 0 && iterations == 1) {
            if (newServoPosition < 1 && newServoPosition - 0.3 > 0) {
                newServoPosition = newServoPosition - 0.3;
                servo.setServoPos(newServoPosition);
            }
        }
        if (iterations > 100) {
            iterations = 0;
        }


        telemetry.addData("currentServoPosition", currentServoPosition);
        telemetry.addData("newServoPosition", newServoPosition);
        telemetry.addData("leftTrigger", gamepad1.left_trigger);
        telemetry.addData("rightTrigger", gamepad1.right_trigger);
        telemetry.addData("iterations", iterations);
        telemetry.update();
    }
}
/*
1. set CR to reverse direction
2. set opmode so left trigger sets position of servo and right trigger makes 0 off and 1 fully on
 */