package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Color Sensor Testing")
public class ColorSensor extends LinearOpMode {
    private NormalizedColorSensor sensor;

    @Override
    public void runOpMode() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color1");

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            float hue = hsvValues[0];

            if (hue >= 90 && hue < 170) {
                telemetry.addData("Detected Color", "GREEN");
                gamepad1.setLedColor(0.0, 1.0, 0.0, 3000);
                gamepad2.setLedColor(0.0, 1.0, 0.0, 3000);
            } else if (hue >= 230 && hue < 250) {
                telemetry.addData("Detected Color", "PURPLE");
                gamepad1.setLedColor(0.5, 0.0, 0.5, 3000);
                gamepad2.setLedColor(0.5, 0.0, 0.5, 3000);
            } else {
                telemetry.addData("Detected Color", "UNKNOWN");
                gamepad1.setLedColor(0.0, 0.0, 1.0, 3000);
                gamepad2.setLedColor(0.0, 0.0, 1.0, 3000);
            }

            telemetry.addData("Hue", "%.2f", hue);
            telemetry.update();
        }
    }
}