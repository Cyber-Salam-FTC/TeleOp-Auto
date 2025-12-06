package org.firstinspires.ftc.teamcode.pedropathing;

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

public class EveryBotConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8)
            .forwardZeroPowerAcceleration(1.0)
            .lateralZeroPowerAcceleration(1.0)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0.0000000004,
                    0.02,
                    0.02
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.2,
                    0.2,
                    0,
                    0.03
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0.001,
                    0.00007,
                    0.6,
                    0.05
            ))
            .centripetalScaling(0.009);
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
            .forwardPodY(5.5)
            .strafePodX(5.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpointComputer")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static MecanumConstants drivetrain = new MecanumConstants()
            .maxPower(0.4)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(64.14651)
            .yVelocity(53.804);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(drivetrain)
                .build();
    }
}

