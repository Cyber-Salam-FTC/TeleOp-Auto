package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cybersalam.hardware.MecanumDrive;

@TeleOp(name = "Cyber Salam TeleOp")
public class MainOp extends OpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor intake, outtake;
    private Servo door;

    private NormalizedColorSensor sensor;
    private String lastDetectedColor = "NONE";

    private enum RobotState {
        RIGHT_AND_FORWARD,
        LEFT_AND_FORWARD,
        RIGHT_AND_BACKWARD,
        LEFT_AND_BACKWARD,
        STOPPED
    };

    double forward, strafe, rotate;

    MecanumDrive drive = new MecanumDrive();

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color1");
        door = hardwareMap.get(Servo.class, "door");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "outtake");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        drive.init(hardwareMap);

        door.setPosition(0);
    }

    public void colorSense() {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        float hue = hsvValues[0];

        float red = (float)colors.red;
        float green = (float)colors.green;
        float blue = (float)colors.blue;

        String currentDetectedColor = "UNKNOWN";

        telemetry.addData("A", "%.3f", colors.alpha);
        telemetry.addData("R", "%.3f", colors.red);
        telemetry.addData("G", "%.3f", colors.green);
        telemetry.addData("B", "%.3f", colors.blue);
        telemetry.addData("HUE", "%.0f", hue);

        if (hue >= 105 && hue < 190) {
            currentDetectedColor = "GREEN";
        } else if (hue >= 206 && hue < 320) {
            currentDetectedColor = "PURPLE";
        }

        telemetry.addData("Detected Color", currentDetectedColor);

        if (!currentDetectedColor.equals(lastDetectedColor)) {
            if (currentDetectedColor.equals("GREEN")) {
                gamepad1.setLedColor(0.0f, 1.0f, 0.0f, 3000);
                gamepad2.setLedColor(0.0f, 1.0f, 0.0f, 3000);
            } else if (currentDetectedColor.equals("PURPLE")) {
                gamepad1.setLedColor(0.5f, 0.0f, 0.5f, 3000);
                gamepad2.setLedColor(0.5f, 0.0f, 0.5f, 3000);
            } else {
                gamepad1.setLedColor(0.0f, 0.0f, 0.0f, 0);
                gamepad2.setLedColor(0.0f, 0.0f, 0.0f, 0);
            }
            lastDetectedColor = currentDetectedColor;
        }
    }

    @Override
    public void loop() {

        forward = gamepad1.right_trigger - gamepad1.left_trigger;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.drive(forward, strafe, rotate);

        telemetry.addData("Right Trigger Value", gamepad1.right_trigger);
        telemetry.addData("Left Trigger Value", gamepad1.left_trigger);
        telemetry.addData("Left Stick X Value", gamepad1.left_stick_x);


        colorSense();

//        intake should always stay moving

        intake.setPower(1);

//        outtake


        outtake.setPower(gamepad2.right_stick_y);

//        door

//        open
        if (gamepad2.cross) {
            door.setPosition(0);
        }
//        closed
        if (gamepad2.circle) {
            door.setPosition(0.3);
        }


        telemetry.update();
    }

}
