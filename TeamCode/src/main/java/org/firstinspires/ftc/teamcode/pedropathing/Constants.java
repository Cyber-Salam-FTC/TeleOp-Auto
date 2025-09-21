package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.Drivetrain;
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

/**
 * This class contains all the configuration constants for the PedroPathing library.
 * The `createFollower` method uses the `FollowerBuilder` to construct a fully
 * configured `Follower` object based on these constants.
 */
public class Constants {
    // Defines constants for the Follower's behavior, like max power and PID gains.
    public static FollowerConstants followerConstants = new FollowerConstants();

    // Defines the constraints for paths, such as maximum power and speed.
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // Defines the constants for the Pinpoint localization system.
    // Make sure these values match your robot's physical setup.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpointComputer")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // Defines the constants for your robot's mecanum drivetrain.
    // Ensure the motor names and directions are correct for your setup.
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

    /**
     * Creates a new Follower object using the FollowerBuilder pattern.
     * This is the correct and modern way to initialize the Follower.
     *
     * @param hardwareMap The hardware map from the OpMode, used to find the motors and sensors.
     * @return A fully configured Follower object.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(drivetrain)
                .build();
    }
}
