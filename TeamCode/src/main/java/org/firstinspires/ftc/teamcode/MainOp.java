package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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
public class MainOp extends OpMode {

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
    int delta;
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
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        rotor = hardwareMap.get(DcMotor.class, "rotor");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        door.setDirection(Servo.Direction.REVERSE);


        rotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        delta = 96;
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

        intake.setPower(1);


        if (gamepad2.triangle) {
            outtake.setPower(0);
        }

        if (gamepad2.a) {
            door.setPosition(0);
        }

        if (gamepad2.b) {
            door.setPosition(0.15);
        }

        rightBumper = gamepad1.right_bumper;


        //        iterations++;
//
       if (rightBumper && rotor.getCurrentPosition()<delta) {
            rotor.setTargetPosition(delta);
            rotor.setPower(0.5);
        }else if(rotor.getCurrentPosition()>=delta){
           rotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       }
//        if (iterations > 50) {
//            iterations = 0;
//        }

//        if (rightBumper) {
//            rotor.setPower(0.1);
//        }
//
//        if (!rightBumper) {
//            rotor.setPower(0);
//        }

//        dpad controls for outtake

//        if (gamepad2.dpad_up) {
//            outtake.setPower(0.9);
//        }if (gamepad2.dpad_down) {
//            outtake.setPower(0.75);
//        }if (gamepad2.dpad_right) {
//        }if (gamepad2.dpad_left) {
//            outtake.setPower(0.3);
//        }

//        if (gamepad2.dpad_up) {
//            outtake.setPower(0.9);
//        }if (gamepad2.dpad_down) {
//            outtake.setPower(0.7);
//        }

        if (gamepad1.dpad_up) {
            outtake.setVelocity(2520);
        }

        if (gamepad1.dpad_down) {
            outtake.setVelocity(1400);
        }

        rotor.setPower(gamepad2.right_stick_y * 0.3);

        telemetry.addData("iterations", iterations);
        telemetry.addData("Rotor Position", rotor.getCurrentPosition());
        telemetry.addData("Rotor Target", rotor.getTargetPosition());
        telemetry.addData("Outtake Power", outtake.getPower());

        telemetry.update();
    }

}