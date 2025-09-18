package org.firstinspires.ftc.teamcode;

import com.bylazar.field.Line;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "Auto Testing Cyber Salam FTC")
public class Auto extends OpMode {

    private Follower follower;
    private PathChain path;

    @Override
    public void init() {



        Pose startPose = new Pose(0, 48, 0);

        // Pass the hardware map to the createFollower method



        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

//        Pose start = new Pose(0, 0, 0);
        Pose middle = new Pose(0, 0, 0);
        Pose end = new Pose(0, 0, 0);

        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, middle))
                .setLinearHeadingInterpolation(startPose.getHeading(), middle.getHeading())
                .build();
    }

    @Override
    public void start() {
        follower.update();
        follower.followPath(path);
    }

    @Override
    public void loop() {
        follower.update();
    }
}