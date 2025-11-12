package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cybersalam.hardware.MecanumDrive;

@TeleOp(name = "Cyber Salam TeleOp")
public class MainOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor intake, rotor;
    private DcMotorEx outtake;
    private Servo door;

    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;

    private NormalizedColorSensor sensor;
    private String lastDetectedColor = "NONE";

    private final int[] ROTOR_PRESETS = {0, 48, 96, 144, 192, 240, 288};
    private int currentPresetIndex = 0;

    private enum RobotState {
        RIGHT_AND_FORWARD,
        LEFT_AND_FORWARD,
        RIGHT_AND_BACKWARD,
        LEFT_AND_BACKWARD,
        STOPPED
    };

    double forward, strafe, rotate;

    boolean leftBumper, rightBumper;
    int rotorPosition;
    MecanumDrive drive = new MecanumDrive();

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color1");
        door = hardwareMap.get(Servo.class, "door");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        rotor = hardwareMap.get(DcMotor.class, "rotor");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        door.setDirection(Servo.Direction.REVERSE);

        rotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotor.setDirection(DcMotor.Direction.REVERSE);
        rotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        drive.init(hardwareMap);

        door.setPosition(0);

        rotorPosition = 0;

        waitForStart();

        while (opModeIsActive()) {

            forward = gamepad1.right_trigger - gamepad1.left_trigger;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            drive.drive(forward, strafe, rotate);

            // colorSense();

            intake.setPower(1);

            if (gamepad2.triangle) {
                outtake.setVelocity(0);
            }

            if (gamepad2.a) {
                door.setPosition(0);
            }

            if (gamepad2.b) {
                door.setPosition(0.25);
            }

            if (gamepad2.ps) {
                intake.setPower(-1);
            }

            if (gamepad2.dpad_left) {
                intake.setPower(1);
            }

            if (gamepad2.dpad_up) {
                outtake.setVelocity(2200);
            }

            if (gamepad2.dpad_down) {
                outtake.setVelocity(1800);
            }

            rightBumper = gamepad2.right_bumper;
            leftBumper = gamepad2.left_bumper;

            boolean rightBumperPressed = rightBumper && !prevRightBumper;
            boolean leftBumperPressed = leftBumper && !prevLeftBumper;

            if (rightBumperPressed || leftBumperPressed) {
                if (rotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (rightBumperPressed) {
                    currentPresetIndex++;
                }
                if (leftBumperPressed) {
                    currentPresetIndex--;
                }

                currentPresetIndex = Math.max(0, Math.min(ROTOR_PRESETS.length - 1, currentPresetIndex));
                rotorPosition = ROTOR_PRESETS[currentPresetIndex];

                rotor.setTargetPosition(rotorPosition);
                rotor.setPower(1.0);
            }

            double rotorManualPower = -gamepad2.right_stick_y * 0.3;

            if (Math.abs(rotorManualPower) > 0.1) {
                if (rotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    rotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                rotor.setPower(rotorManualPower);
            } else if (rotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                rotor.setPower(0);
            }

            prevRightBumper = rightBumper;
            prevLeftBumper = leftBumper;

            if (rotor.getCurrentPosition() == 288) {
                rotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Current Preset Index", currentPresetIndex);
        telemetry.addData("Right Bumper Pressed", rightBumper && !prevRightBumper);
        telemetry.addData("Left Bumper Pressed", leftBumper && !prevLeftBumper);
        telemetry.addData("Rotor Mode", rotor.getMode());
        telemetry.addData("Rotor Target", rotorPosition);
        telemetry.addData("Rotor Current Position", rotor.getCurrentPosition());
        telemetry.addData("Outtake Velocity", outtake.getVelocity());
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.update();
    }
//    public void colorSense() {
//        NormalizedRGBA colors = sensor.getNormalizedColors();
//        float[] hsvValues = new float[3];
//        Color.colorToHSV(colors.toColor(), hsvValues);
//
//        float hue = hsvValues[0];
//
//
//        float red = (float)colors.red;
//        float green = (float)colors.green;
//        float blue = (float)colors.blue;
//
//        String currentDetectedColor = "UNKNOWN";
//
//        telemetry.addData("A", "%.3f", colors.alpha);
//        telemetry.addData("R", "%.3f", colors.red);
//        telemetry.addData("G", "%.3f", colors.green);
//        telemetry.addData("B", "%.3f", colors.blue);
//        telemetry.addData("HUE", "%.0f", hue);
//
//        if (hue >= 105 && hue < 190) {
//            currentDetectedColor = "GREEN";
//        } else if (hue >= 206 && hue < 320) {
//            currentDetectedColor = "PURPLE";
//        }
//
//        telemetry.addData("Detected Color", currentDetectedColor);
//
//        if (!currentDetectedColor.equals(lastDetectedColor)) {
//            if (currentDetectedColor.equals("GREEN")) {
//                gamepad1.setLedColor(0.0f, 1.0f, 0.0f, 3000);
//                gamepad2.setLedColor(0.0f, 1.0f, 0.0f, 3000);
//            } else if (currentDetectedColor.equals("PURPLE")) {
//                gamepad1.setLedColor(0.5f, 0.0f, 0.5f, 3000);
//                gamepad2.setLedColor(0.5f, 0.0f, 0.5f, 3000);
//            } else {
//                gamepad1.setLedColor(0.0f, 0.0f, 0.0f, 0);
//                gamepad2.setLedColor(0.0f, 0.0f, 0.0f, 0);
//            }
//            lastDetectedColor = currentDetectedColor;
//        }
//    }

}