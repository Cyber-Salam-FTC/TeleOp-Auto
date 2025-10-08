package org.firstinspires.ftc.teamcode.testing.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServoExamples extends OpMode {
    org.firstinspires.ftc.teamcode.Mechanisms.servo servo = new org.firstinspires.ftc.teamcode.Mechanisms.servo();
    double leftTrigger, rightTrigger;

    @Override
    public void init() {
        // Initialize servos
        servo.init(hardwareMap);
        leftTrigger = 0.0;
        rightTrigger = 0.0;
    }
    @Override
    public void loop() {
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;

        servo.setServoRot(rightTrigger);
        servo.setServoRot(leftTrigger);
        // Handle gamepad input to control servos
        if (gamepad1.a) {
            servo.setServoPos(-1.0);
        } else {
            servo.setServoPos(1.0);
        }
        if (gamepad1.b) {
            servo.setServoRot(1.0);
        } else {
            servo.setServoRot(0);
        }
        if(gamepad1.b){
            servo.setServoRot(1.0);
        }
        else {
            servo.setServoRot(0);
        }
    }
}
/*
1. set CR to reverse direction
2. set opmode so left trigger sets position of servo and right trigger makes 0 off and 1 fully on
 */