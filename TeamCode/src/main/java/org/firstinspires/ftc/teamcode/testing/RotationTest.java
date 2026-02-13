//package org.firstinspires.ftc.teamcode.testing;
//
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawCurrent;
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawCurrentAndHistory;
//import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;
//
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.pedropathing.Constants;
//
//@Autonomous(name = "Rotation test")
//public class RotationTest extends OpMode {
//
//    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
//    private final Pose interPose = new Pose(24, -24, Math.toRadians(90));
//    private final Pose endPose = new Pose(24, 24, Math.toRadians(45));
//
//    private PathChain triangle;
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the Panels.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        drawCurrentAndHistory();
//
//        if (follower.atParametricEnd()) {
//            follower.followPath(triangle, true);
//        }
//    }
//
//    @Override
//    public void init() {}
//
//    @Override
//    public void init_loop() {
//        follower.update();
//        drawCurrent();
//    }
//
//    /** Creates the PathChain for the "triangle".*/
//    @Override
//    public void start() {
//        follower.setStartingPose(startPose);
//
//        triangle = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, interPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
//                .addPath(new BezierLine(interPose, endPose))
//                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
//                .addPath(new BezierLine(endPose, startPose))
//                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
//                .build();
//
//        follower.update();
//        follower.followPath(triangle);
//    }
//}