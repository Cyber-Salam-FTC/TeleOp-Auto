package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.cybersalam.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "Auto Testing Cyber Salam FTC")
public class Auto extends OpMode {

    private Follower follower;
    private PathChain path;
    RobotHardware hardware = new RobotHardware();

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    @Override
    public void init() {
        hardware.init(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        Pose startPose = new Pose(0, 0, 0);
        Pose end = new Pose(30, 92, Math.toRadians(90));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, end))
                .setLinearHeadingInterpolation(startPose.getHeading(), end.getHeading())
                .build();
    }

    @Override
    public void start() {
        follower.followPath(path);
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }
}