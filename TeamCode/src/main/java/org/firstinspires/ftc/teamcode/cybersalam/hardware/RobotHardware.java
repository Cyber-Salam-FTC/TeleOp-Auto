package org.firstinspires.ftc.teamcode.cybersalam.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Mat;

public class RobotHardware {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private IMU imu;


    int DirectionCase;

    public void init(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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