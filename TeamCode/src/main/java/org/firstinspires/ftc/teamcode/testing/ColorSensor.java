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
        try {
            sensor = hardwareMap.get(NormalizedColorSensor.class, "color1");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not find color sensor 'color1'. Check your hardware map!");
            telemetry.update();
            return;
        }

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            float hue = hsvValues[0];

            // Convert to float for setLedColor compatibility
            float red = (float)colors.red;
            float green = (float)colors.green;
            float blue = (float)colors.blue;

            // Display raw sensor data for easy debugging
            telemetry.addData("A", "%.3f", colors.alpha);
            telemetry.addData("R", "%.3f", colors.red);
            telemetry.addData("G", "%.3f", colors.green);
            telemetry.addData("B", "%.3f", colors.blue);
            telemetry.addData("HUE", "%.0f", hue); // The key value for color identification

            // Color detection logic
            if (hue >= 105 && hue < 190         ) {
                telemetry.addData("Detected Color", "GREEN");
            } else if (hue >= 220 && hue < 320) { // Expanded the range for Purple/Magenta
                telemetry.addData("Detected Color", "PURPLE");
            } else {
                telemetry.addData("Detected Color", "UNKNOWN");
            }

            telemetry.update();

            // Setting the controller LED colors (requires float values)
            gamepad1.setLedColor(red, green, blue, 3000);
            gamepad2.setLedColor(red, green, blue, 3000);
        }
    }
}
