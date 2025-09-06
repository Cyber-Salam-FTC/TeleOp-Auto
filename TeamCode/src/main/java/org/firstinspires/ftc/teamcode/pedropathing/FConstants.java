package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.LConstants;
import com.acmerobotics.dashboard.config.Config;

@Config
public class FConstants {
    public static final FollowerConstants followerConstants;

    public static double translational_kP = 0.02;
    public static double translational_kD = 0.001;
    public static double heading_kP = 0.01;
    public static double heading_kD = 0.0005;

    static {
        followerConstants = new FollowerConstants();
        followerConstants.localizers = Localizers.DRIVE_ENCODERS;
        followerConstants.mass = 7.6;

        // These values will be applied manually in your OpMode
    }
}