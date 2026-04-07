package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous()
public class teleopstartingauto extends LinearOpMode {
    private PathChain path;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        Pose pos1 = new Pose(72, 8, Math.toRadians(90));
        Pose pos2 = new Pose(100, 72, Math.toRadians(90));

        path = follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
                .build();

        waitForStart();

        while (opModeIsActive()) {
            follower.followPath(path);
            follower.update();

            if (!follower.isBusy()) {
                requestOpModeStop();
            }
        }
    }
}
