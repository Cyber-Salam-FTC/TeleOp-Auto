package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.cybersalam.hardware.MecanumDrive;

@TeleOp(name = "LimeLight Test")
public class LimeLightTest extends OpMode {

    private Limelight3A limelight3A;
    private IMU imu;

    private DcMotorEx outtake;
    private double distance;

    double forward, strafe, rotate;

    private Servo door;


    @Override
    public void init() {


        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        outtake = hardwareMap.get(DcMotorEx.class, "shooter");
        limelight3A.pipelineSwitch(0);

        door = hardwareMap.get(Servo.class, "door");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        door.setDirection(Servo.Direction.REVERSE);


    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {


        MecanumDrive drive = new MecanumDrive();
        drive.init(hardwareMap);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botpose.toString());
            telemetry.addData("Outtake Velocity", outtake.getVelocity());
        }

        forward = gamepad1.right_trigger - gamepad1.left_trigger;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.drive(forward, strafe, rotate);

        outtake.setVelocity(getVelocity(getDistanceFromTag(llResult.getTa())));


    }

    public double getDistanceFromTag(double ta) {
        double scale = 5604.074;
        double distance = Math.sqrt(scale / ta);
        return distance;
    }


    public double getVelocity(double dist) {
        double a = 0.0140252;
        double velocity = ((a*(Math.pow(dist, 2))) + (7.1669*dist) + 1300.8906);
        return velocity;
    }


}