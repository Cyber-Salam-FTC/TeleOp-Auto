package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Rotation Test")
public class RotationTest extends OpMode {
    private Follower follower;
    private Pose PoseA;
    private Pose PoseB;
    private PathChain path;

    @Override
    public void init() {
        PoseA = new Pose(0, 0, Math.toRadians(90));
        PoseB = new Pose(0, 0, 0);

        path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(PoseA, PoseB)))
                .build();
    }

    @Override
    public void loop() {
        follower.followPath(path);
    }
}
