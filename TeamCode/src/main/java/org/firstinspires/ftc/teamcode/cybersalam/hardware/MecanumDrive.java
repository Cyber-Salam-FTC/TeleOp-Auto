//package org.firstinspires.ftc.teamcode.cybersalam.hardware;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//
//public class MecanumDrive {
//
//    private IMU imu;
//
//    public void init(HardwareMap hwMap){
//        RobotHardware hardware = new RobotHardware();
//
//        imu = hwMap.get(IMU.class, "pinpoint");
//
//        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
//        );
//
//        imu.initialize(new IMU.Parameters(RevOrientation));
//
//
//    }
//
//
//    public void drive(double forward, double strafe, double rotate) {
//        double leftPower = throtle
//    }
//
//}
