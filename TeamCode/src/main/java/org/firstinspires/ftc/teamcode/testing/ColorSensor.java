package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Color Sensor Testing")
public class ColorSensor extends LinearOpMode {
    private NormalizedColorSensor sensor;

    public void runOpMode() {
            sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");


            waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            // Get the hue value from the HSV array
            float hue = hsvValues[0];

            double red = colors.red;
            double green = colors.green;
            double blue = colors.blue;


            if (hue >= 105 && hue < 165) {
                telemetry.addData("Detected Color", "GREEN");
            } else if (hue >= 260 && hue < 300) {
                telemetry.addData("Detected Color", "PURPLE");
            } else if ((hue >= 0 && hue < 15) || (hue >= 345 && hue <= 360)) {
                telemetry.addData("Detected Color", "RED");
            } else {
                telemetry.addData("Detected Color", "UNKNOWN");
            }

            telemetry.update();


//           Setting the controller to the purple or green colors based on color sensor value

            gamepad1.setLedColor(red, green, blue, 3000);
            gamepad2.setLedColor(red, green, blue, 3000);

        }
    }
}
