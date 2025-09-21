package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArcadeDrive {
    private DcMotor frontleftMotor, BackleftMotor, frontrightMotor, BackrightMotor;

    public void init(HardwareMap hwMap){
        frontleftMotor = hwMap.get(DcMotor.class, "frontleftMotor");
        BackleftMotor = hwMap.get(DcMotor.class, "backleftMotor");
        frontrightMotor = hwMap.get(DcMotor.class, "frontrightMotor");
        BackrightMotor = hwMap.get(DcMotor.class, "backrightMotor");

        frontleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackleftMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    public void drive(double throttle, double spin ){
        double leftPower = throttle + spin;
        double rightPower = throttle - spin;
        double largestPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (largestPower > 1.0) {
            leftPower /= largestPower;
            rightPower /= largestPower;
        }

        frontleftMotor.setPower(leftPower);
        BackleftMotor.setPower(leftPower);
        frontrightMotor.setPower(rightPower);
        BackrightMotor.setPower(rightPower);

    }
}
