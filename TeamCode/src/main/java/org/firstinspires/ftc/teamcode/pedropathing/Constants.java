package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(1.0)
            .lateralZeroPowerAcceleration(1.0)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.05,
                    0.0009,
                    0.02,
                    0.02
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.1,
                    0.003,
                    0,
                    0.02
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0.006,
                    0.01,
                    0,
                    0.02
            ))
            .centripetalScaling(0.001);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            1.0,
            2.0,
            0.009,
            5,
            60,
            10,
            50
    );
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4)
            .strafePodX(7.0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpointComputer")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants drivetrain = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(drivetrain)
                .build();
    }
}
