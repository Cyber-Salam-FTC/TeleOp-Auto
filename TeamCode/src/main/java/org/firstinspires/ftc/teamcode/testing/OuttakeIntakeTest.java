package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeIntakeTest extends OpMode {
    private DcMotor intake;
    private DcMotor outtake;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
    }

    @Override
    public void loop() {
//        RIGHT_STICK_Y RUNS INTAKE
        if (gamepad1.right_stick_y > 0) {
            intake.setPower(gamepad1.right_stick_y);
        }
//        LEFT_STICK_Y RUNS MAG
        if (gamepad1.left_stick_y > 0) {
            outtake.setPower(gamepad1.left_stick_y);
        }
    }
}
