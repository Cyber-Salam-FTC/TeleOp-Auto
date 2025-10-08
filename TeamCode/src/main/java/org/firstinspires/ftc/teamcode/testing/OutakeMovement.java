package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Outtake Testing")
public class OutakeMovement  extends OpMode {
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
