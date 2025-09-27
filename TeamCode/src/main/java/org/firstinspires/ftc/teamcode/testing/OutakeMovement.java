package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OutakeMovement  extends OpMode {
    private DcMotor outake;

    @Override
    public void init() {
        outake = hardwareMap.get(DcMotor.class, "outtake");
    }

    @Override
    public void loop() {
        outake.setPower(1);
    }
}
