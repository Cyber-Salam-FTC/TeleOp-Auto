/*package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Outtake Testing")
public class OutakeMovement  extends OpMode {
    boolean leftBumper, rightBumper;
    double currentServoPosition, newServoPosition;
    double iterations;
    double delta;

    @Override
    public void init() {
        // Initialize servos
        OutakeMovement.init(hardwareMap);
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
   private DcMotor outake;


    @Override
    public void init() {
        outake = hardwareMap.get(DcMotor.class, "outtake");
        outake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        outake.setPower(1);
    }

}
*/