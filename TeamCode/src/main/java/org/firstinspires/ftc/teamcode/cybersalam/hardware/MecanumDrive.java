// File: org.firstinspires.ftc.teamcode.cybersalam.hardware.MecanumDrive.java

package org.firstinspires.ftc.teamcode.cybersalam.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU; // Import the BNO055IMU class
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private BNO055IMU imu;

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

        // Get the BNO055IMU instead of the generic IMU interface.
        imu = hwMap.get(BNO055IMU.class, "imu");

        // The BNO055IMU requires a different parameter object.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(parameters);
    }

    public BNO055IMU getIMU() {
        return imu;
    }

    public void drive(double forward, double strafe, double rotate) {
        double leftFrontPower = forward + strafe + rotate;
        double leftRearPower = forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightRearPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));

        leftFront.setPower(maxSpeed * (leftFrontPower / maxPower));
        leftRear.setPower(maxSpeed * (leftRearPower / maxPower));
        rightFront.setPower(maxSpeed * (rightFrontPower / maxPower));
        rightRear.setPower(maxSpeed * (rightRearPower / maxPower));
    }

    // You can't use getRobotYawPitchRollAngles() with BNO055IMU.
    // You must use imu.getAngularOrientation() instead.
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Use imu.getAngularOrientation() for BNO055IMU.
        theta = AngleUnit.normalizeRadians(theta -
                imu.getAngularOrientation().firstAngle);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }
}