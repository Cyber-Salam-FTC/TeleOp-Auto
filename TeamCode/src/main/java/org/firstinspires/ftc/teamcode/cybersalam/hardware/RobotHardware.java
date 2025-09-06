package org.firstinspires.ftc.teamcode.cybersalam.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    public void init(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
    }

    public void leftFrontSpeed(double speed) {
        leftFront.setPower(speed);
    }

    public void leftRearSpeed(double speed) {
        leftRear.setPower(speed);
    }
    public void rightFrontSpeed(double speed) {
        rightFront.setPower(speed);
    }
    public void rightRearSpeed(double speed) {
        rightRear.setPower(speed);
    }
}
