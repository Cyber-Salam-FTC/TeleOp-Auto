package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp
public class LimeLightTest extends OpMode {

    private Limelight3A limelight3A;
    private IMU imu;

    private DcMotorEx outtake;
    private double distance;


    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        limelight3A.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
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
        }



        outtake.setVelocity(getVelocity(getDistanceFromTag(llResult.getTa())));

    }

    public double getDistanceFromTag(double ta) {
        double scale = 5604.074;
        double distance = Math.sqrt(scale / ta);
        return distance;
    }


    public double getVelocity(double dist) {
        double a = 0.0140252;
        double velocity = (a*(Math.pow(dist, 2))) + (7.1669*dist) + 1265.8906;
        return velocity;
    }
}