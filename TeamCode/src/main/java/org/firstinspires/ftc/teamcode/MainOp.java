package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.cybersalam.hardware.RobotHardware;

@TeleOp(name = "Cyber Salam TeleOp")
public class MainOp extends OpMode {

    RobotHardware hardware = new RobotHardware();
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    private enum RobotState {
        RIGHT_AND_FORWARD,
        LEFT_AND_FORWARD,
        RIGHT_AND_BACKWARD,
        LEFT_AND_BACKWARD,
        STOPPED
    };

    @Override
    public void init() {
        // This is a good place to put hardware-specific logic.
        hardware.init(hardwareMap);

        // It is a good practice to also get motor objects directly
        // to make sure they are properly mapped.
        try {
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            leftRear = hardwareMap.get(DcMotor.class, "leftRear");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightRear = hardwareMap.get(DcMotor.class, "rightRear");

            // You might need to reverse one side of the motors so your robot drives straight.
            // For example, if the right side motors spin the wrong way, you can uncomment these lines:
            // rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            // rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        } catch (Exception e) {
            // This will tell you if there's a hardware configuration error!
            telemetry.addData("Error", "Motor initialization failed. Check your hardware map names!");
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        // Now using the right and left trigger's values for telemetry
        telemetry.addData("Right Trigger Value", gamepad1.right_trigger);
        telemetry.addData("Left Trigger Value", gamepad1.left_trigger);
        telemetry.addData("Left Stick X Value", gamepad1.left_stick_x);
        telemetry.update();

        // Forward movements use the right trigger
        if (gamepad1.left_trigger > 0.5 && gamepad1.left_stick_x > 0) {
            leftFront.setPower(1);
            rightFront.setPower(0.5);
            leftRear.setPower(1);
            rightRear.setPower(0.5);
        } else if (gamepad1.left_trigger > 0.5 && gamepad1.left_stick_x < 0) {
            leftFront.setPower(0.5);
            rightFront.setPower(1);
            leftRear.setPower(0.5);
            rightRear.setPower(1);
            // Backward movements use the left trigger
        } else if (gamepad1.right_trigger > 0.5 && gamepad1.left_stick_x > 0) {
            leftFront.setPower(-1);
            rightFront.setPower(-0.5);
            leftRear.setPower(-1);
            rightRear.setPower(-0.5);
        } else if (gamepad1.right_trigger > 0.5 && gamepad1.left_stick_x < 0) {
            leftFront.setPower(-0.5);
            rightFront.setPower(-1);
            leftRear.setPower(-0.5);
            rightRear.setPower(-1);
        } else {
            // This is the important part! If none of the other conditions are met,
            // the robot stops moving.
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }
    }

    @Override
    public void stop() {
        // This method is called when the OpMode is stopped.
        // It's a final safety check to make sure the motors stop.
        // The isBusy() check is typically only for autonomous mode.
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
