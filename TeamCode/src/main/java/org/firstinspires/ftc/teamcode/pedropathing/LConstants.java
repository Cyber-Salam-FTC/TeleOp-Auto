package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.localization.constants.DriveEncoderConstants;
import com.pedropathing.localization.Encoder;

// This acts as a method of updating DriveEncoderLocalizer without direct access to it.
public class LConstants { // This is how we change DriveEncoderLocalizer.
    static {
        DriveEncoderConstants.forwardTicksToInches = 5.953;
        DriveEncoderConstants.strafeTicksToInches = 0.7666;
        DriveEncoderConstants.turnTicksToInches = 28.7231;

        DriveEncoderConstants.robot_Width = 16;
        DriveEncoderConstants.robot_Length = 16;

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.FORWARD;
    }
}
