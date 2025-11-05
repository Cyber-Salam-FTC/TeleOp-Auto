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
//    private double outtakePower = 1;

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

    boolean leftBumper, rightBumper;
    double iterations;
    int delta, rotorPosition;
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


        rotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        leftBumper = false;
        rightBumper = false;

        iterations = 0;
        delta = 48;
        rotorPosition = 0;

        while (opModeIsActive()) {

            forward = gamepad1.right_trigger - gamepad1.left_trigger;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            drive.drive(forward, strafe, rotate);

            telemetry.addData("Right Trigger Value", gamepad1.right_trigger);
            telemetry.addData("Left Trigger Value", gamepad1.left_trigger);
            telemetry.addData("Left Stick X Value", gamepad1.left_stick_x);


            colorSense();

            intake.setPower(1);


            if (gamepad2.triangle) {
                outtake.setVelocity(0);
            }

            if (gamepad2.a) {
                door.setPosition(0);
            }

            if (gamepad2.b) {
                door.setPosition(0.15);
            }

            rightBumper = gamepad2.right_bumper;
            leftBumper = gamepad2.left_bumper;

            if (rightBumper) {
                rotorPosition += delta;
            }
            if (leftBumper) {
                rotorPosition -= delta;
            }
            rotor.setTargetPosition(rotorPosition);
            rotor.setPower(1);
            rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            iterations = 0;
           while (rotor.isBusy() && iterations < 40) {
               telemetry.addData("rotorPosition", rotorPosition);
               telemetry.addData("Current rotor position", rotor.getCurrentPosition());
               telemetry.update();
               sleep(25);
               iterations++;
           }

            if (gamepad1.dpad_up) {
                outtake.setVelocity(2520);
            }

            if (gamepad1.dpad_down) {
                outtake.setVelocity(1400);
            }

            rotor.setPower(gamepad2.right_stick_y * 0.3);

            telemetry.addData("rightBumper", rightBumper);
            telemetry.addData("Rotor Position", rotor.getCurrentPosition());
            telemetry.addData("Rotor Target", rotor.getTargetPosition());
            telemetry.addData("Outtake Power", outtake.getPower());

            telemetry.update();
        }
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



}