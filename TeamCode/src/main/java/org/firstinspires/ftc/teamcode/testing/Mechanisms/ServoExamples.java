package org.firstinspires.ftc.teamcode.testing.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp (name = "Servo Test")
public class ServoExamples extends OpMode {
    org.firstinspires.ftc.teamcode.testing.Mechanisms.servo servo = new org.firstinspires.ftc.teamcode.testing.Mechanisms.servo();
    double leftTrigger, rightTrigger;

    @Override
    public void init() {
        // Initialize servos
        servo.init(hardwareMap);
        leftTrigger = 0.0;
//        rightTrigger = 0.0;
    }
    @Override
    public void loop() {
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;

//        servo.setServoRot(rightTrigger);
        if (leftTrigger > 0)
            servo.setServoRot(leftTrigger);
        else if (rightTrigger > 0)
            servo.setServoRot(-rightTrigger);
        else
            servo.setServoRot(0);
        // Handle gamepad input to control servos
        if (gamepad1.cross) {
            servo.setServoPos(0.5);
        } else {
            servo.setServoPos(0);
        }
//        if (gamepad1.circle) {
//            servo.setServoRot(1);
//        } else {
//            servo.setServoRot(0.5);
//        }

        telemetry.addData("leftTrigger", leftTrigger);
        telemetry.addData("rightTrigger", -rightTrigger);
        telemetry.addData("cross", gamepad1.cross);
//        telemetry.addData("circle", gamepad1.circle);
        telemetry.update();
    }
}
/*
1. set CR to reverse direction
2. set opmode so left trigger sets position of servo and right trigger makes 0 off and 1 fully on
 */