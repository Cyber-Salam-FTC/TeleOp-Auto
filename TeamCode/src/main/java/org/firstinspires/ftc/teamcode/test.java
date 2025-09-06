package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cybersalam.hardware.RobotHardware;

public class test extends OpMode {

    RobotHardware hardware = new RobotHardware();

    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {
        hardware.leftFrontSpeed();
    }
}