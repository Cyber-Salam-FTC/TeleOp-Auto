package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ArcadeDriveOpMode extends OpMode {
    ArcadeDrive drive=new ArcadeDrive();
    double throttle, spin;

    @Override
    public void init() {
        drive.init(hardwareMap);

    }
    @Override
    public void loop() {
        double throttle = gamepad1.left_stick_y;
         spin = gamepad1.right_stick_x;
        drive.drive(throttle, spin);

    }

}
