package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cybersalam.hardware.RobotHardware;

@TeleOp(name = "Cyber Salam TeleOp")
public class MainOp extends OpMode {

    RobotHardware hardware = new RobotHardware();

    double forward, strafe, rotate;

    double controllerState;



    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = -(gamepad1.left_stick_x);
        strafe = -(gamepad1.left_stick_y);
        rotate = gamepad1.right_stick_x;

//        hardware.driveFieldRelative(forward, strafe, rotate);

    }



}
