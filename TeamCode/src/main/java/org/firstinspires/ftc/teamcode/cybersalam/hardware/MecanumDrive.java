package org.firstinspires.ftc.teamcode.cybersalam.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    // Removed IMU as it's cleaner to handle it in RobotHardware or pass it in if needed

    // Initialize with the motors from RobotHardware
    public void init(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
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

        // Power scaling
        double scale = maxSpeed / maxPower;

        leftFront.setPower(leftFrontPower * scale);
        leftRear.setPower(leftRearPower * scale);
        rightFront.setPower(rightFrontPower * scale);
        rightRear.setPower(rightRearPower * scale);
    }

    // driveFieldRelative requires an IMU object to be passed in or stored.
    // I'm keeping the method simple for now, but you'll need the IMU for this.
    /*
    public void driveFieldRelative(double forward, double strafe, double rotate, double robotHeadingRadians) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - robotHeadingRadians);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }
    */
}