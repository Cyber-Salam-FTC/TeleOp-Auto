package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
<<<<<<< HEAD
//Importing PedroPathing
import com.pedropathing.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.localization.Pose;

@Autonomous(name = "Autonomous Cyber Salam FTC")
public class Auto extends LinearOpMode {

    private DcMotor armbottom;
    private DcMotor armtop;
    private DcMotor intake;
    private DcMotor motorLB;
=======

@Autonomous(name = "MAIN_02092025 (Blocks to Java)")
public class Auto extends LinearOpMode {
    private DcMotor armbottom;
    private DcMotor armtop;
    private DcMotor intake;
    private DcMotor moterLB;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
    private DcMotor motorLF;
    private DcMotor motorRB;
    private DcMotor motorRF;
    private ColorSensor color_REV_ColorRangeSensor;
    private Servo servoclaw;
<<<<<<< HEAD
    //    PedroPathing variables
    private DriveEncoderLocalizer localizer;

    int fposition;
    double fPower;
    int armbottomposition;
=======

    double fPower;
    int armbottomposition;
    int fposition;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97

    /**
     * Telemetry For all motors
     */
    private void TELEMETRY2() {
        telemetry.addData("armbottom", armbottom.getCurrentPosition());
        telemetry.addData("armtop", armtop.getCurrentPosition());
        telemetry.addData("intake", intake.getCurrentPosition());
<<<<<<< HEAD
        telemetry.addData("motorLB", motorLB.getCurrentPosition());
=======
        telemetry.addData("motorLB", moterLB.getCurrentPosition());
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        telemetry.addData("motorLF", motorLF.getCurrentPosition());
        telemetry.addData("motorRB", motorRB.getCurrentPosition());
        telemetry.addData("motorRF", motorRF.getCurrentPosition());
    }

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        armbottom = hardwareMap.get(DcMotor.class, "armbottom");
        armtop = hardwareMap.get(DcMotor.class, "armtop");
        intake = hardwareMap.get(DcMotor.class, "intake");
<<<<<<< HEAD
        motorLB = hardwareMap.get(DcMotor.class, "motorLB");
=======
        moterLB = hardwareMap.get(DcMotor.class, "moterLB");
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");
        color_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "color");
        servoclaw = hardwareMap.get(Servo.class, "servoclaw");

<<<<<<< HEAD

        armbottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armtop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
=======
        armbottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armtop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moterLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        INITIALIZE_MOTORS();
        CLAW_CLOSE();
        waitForStart();
        if (opModeIsActive()) {
<<<<<<< HEAD
//            Update PedroPathing
            localizer.update();
            Pose currentPose = localizer.getPose();
            telemetry.update();
            // Put run blocks here.
            // hanging first specimen
            while (opModeIsActive()) {
                fPower = 1;
=======
            // Put run blocks here.
            // hanging first specimen
            while (opModeIsActive()) {
                fPower = 0.7;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                INITIALIZE_MOTORS();
                armbottomposition = 2800;
                // 5. Open claws and raise arm
                ARM_BOTTOM();
<<<<<<< HEAD
                sleep(600);
=======
                sleep(500);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                fposition = 1000;
                MOVE_FORWARD_OR_BACKWARDS();
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                INITIALIZE_MOTORS();
                armbottomposition = -1000;
                // 12. Raise arm
                ARM_BOTTOM();
<<<<<<< HEAD
                sleep(4500);
=======
                sleep(300);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                CLAW_OPEN();
                break;
            }
            // going to bring one sample to oz
            while (opModeIsActive()) {
<<<<<<< HEAD
                fPower = 0.8;
=======
                fPower = 0.9;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                INITIALIZE_MOTORS();
                fposition = -300;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                armbottomposition = -1700;
                // 12. Raise arm
                ARM_BOTTOM();
                fposition = -1800;
                SHIFT_SIDEWAYS();
                INITIALIZE_MOTORS();
                fposition = 1500;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                fposition = -625;
                SHIFT_SIDEWAYS();
                INITIALIZE_MOTORS();
                fposition = -1700;
                MOVE_FORWARD_OR_BACKWARDS();
                break;
            }
            // Turn around to position for 2nd specimen
            while (opModeIsActive()) {
                fPower = 1;
                INITIALIZE_MOTORS();
                fposition = 200;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                fposition = 2150;
                TURN_RIGHT();
                break;
            }
            while (opModeIsActive()) {
                MOVE_FORWARD_USING_DISTANCE2();
                break;
            }
<<<<<<< HEAD
            while (opModeIsActive()) {
                fPower = 1;
                INITIALIZE_MOTORS();
                armbottomposition = 1150;
                // 12. Raise arm
                ARM_BOTTOM();
                fposition = -350;
=======
            // Pick up the 2nd specimen
            while (opModeIsActive()) {
                fPower = 1;
                INITIALIZE_MOTORS();
                armbottomposition = 1200;
                ARM_BOTTOM();
                fposition = -380;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                MOVE_FORWARD_OR_BACKWARDS();
                CLAW_OPEN();
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                INITIALIZE_MOTORS();
                fposition = 20;
                MOVE_FORWARD_OR_BACKWARDS();
                CLAW_CLOSE();
                break;
            }
<<<<<<< HEAD
=======

>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
            while (opModeIsActive()) {
                fPower = 1;
                INITIALIZE_MOTORS();
                armbottomposition = 400;
                // 12. Raise arm
                ARM_BOTTOM();
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                break;
            }
<<<<<<< HEAD
            // Move to hang the 2nd specimen
            while (opModeIsActive()) {
                fPower = 0.9;
=======
            while (opModeIsActive()) {
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                INITIALIZE_MOTORS();
                fposition = 1100;
                TURN_RIGHT();
                INITIALIZE_MOTORS();
                fposition = 1800;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                armbottomposition = 1500;
                // 12. Raise arm
                ARM_BOTTOM();
                fposition = 1150;
                TURN_RIGHT();
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                break;
            }
            while (opModeIsActive()) {
                MOVE_FORWARD_USING_DISTANCE2();
                break;
            }
            // Hang the 2nd specimen
            while (opModeIsActive()) {
<<<<<<< HEAD
                fPower = 0.9;
                INITIALIZE_MOTORS();
                fposition = -350;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                armbottomposition = -1250;
=======
                fPower = 1;
                INITIALIZE_MOTORS();
                fposition = -250;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                armbottomposition = -1300;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                ARM_BOTTOM();
                sleep(550);
                CLAW_OPEN();
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                break;
            }
<<<<<<< HEAD
            //
            while (opModeIsActive()) {
                fPower = 0.9;
                INITIALIZE_MOTORS();
                fposition = -250;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                armbottomposition = -1400;
=======
            while (opModeIsActive()) {
                fPower = 1;
                INITIALIZE_MOTORS();
                fposition = -200;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                armbottomposition = -1300;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                ARM_BOTTOM();
                fposition = 2150;
                TURN_RIGHT();
                INITIALIZE_MOTORS();
<<<<<<< HEAD
                fposition = 2000;
=======
                fposition = 1800;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                SHIFT_SIDEWAYS();
                break;
            }
            while (opModeIsActive()) {
                MOVE_FORWARD_USING_DISTANCE2();
                break;
            }
<<<<<<< HEAD
            // Pick the 3rd specimen
            while (opModeIsActive()) {
                fPower = 1;
                INITIALIZE_MOTORS();
                armbottomposition = 1000;
                // 12. Raise arm
                ARM_BOTTOM();
                fposition = -350;
=======
            // pick up the 3rd specimen
            while (opModeIsActive()) {
                fPower = 1;
                INITIALIZE_MOTORS();
                armbottomposition = 1100;
                // 12. Raise arm
                ARM_BOTTOM();
                fposition = -380;
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
                MOVE_FORWARD_OR_BACKWARDS();
                CLAW_OPEN();
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                INITIALIZE_MOTORS();
                fposition = 20;
                MOVE_FORWARD_OR_BACKWARDS();
                CLAW_CLOSE();
                INITIALIZE_MOTORS();
                armbottomposition = 400;
                // 12. Raise arm
                ARM_BOTTOM();
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                INITIALIZE_MOTORS();
                fposition = 2150;
                TURN_RIGHT();
                INITIALIZE_MOTORS();
                armbottomposition = 1900;
                // 12. Raise arm
                ARM_BOTTOM();
                fposition = 1800;
                SHIFT_SIDEWAYS();
                MOVE_FORWARD_USING_DISTANCE2();
                break;
            }
<<<<<<< HEAD
            // Go back hang third
=======
            // Go back hand third
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
            while (opModeIsActive()) {
                fPower = 1;
                INITIALIZE_MOTORS();
                fposition = -250;
                MOVE_FORWARD_OR_BACKWARDS();
                INITIALIZE_MOTORS();
                armbottomposition = -1300;
                ARM_BOTTOM();
                sleep(550);
                CLAW_OPEN();
                while (armbottom.isBusy()) {
                    sleep(10);
                }
                break;
            }
        }
    }

    /**
     * Encoder, and brake
     */
    private void INITIALIZE_MOTORS() {
        armbottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armtop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
<<<<<<< HEAD
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
=======
        moterLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Encoder, and brake
     */
    private void INITIALIZE_MOTORS2() {
        armbottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armtop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
<<<<<<< HEAD
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
=======
        moterLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Target postion for the motors
     */
    private void MOVE_FORWARD_OR_BACKWARDS() {
<<<<<<< HEAD
        motorLB.setPower(-fPower);
        motorLF.setPower(-fPower);
        motorRB.setPower(fPower);
        motorRF.setPower(fPower);
        motorLB.setTargetPosition(-fposition);
=======
        moterLB.setPower(-fPower);
        motorLF.setPower(-fPower);
        motorRB.setPower(fPower);
        motorRF.setPower(fPower);
        moterLB.setTargetPosition(-fposition);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setTargetPosition(-fposition);
        motorRB.setTargetPosition(fposition);
        motorRF.setTargetPosition(fposition);
        RUN_TO_POSITION();
        TELEMETRY2();
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void ARM_BOTTOM() {
        armbottom.setPower(-(4 * fPower));
        armbottom.setTargetPosition(armbottomposition);
        armbottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void TURN_RIGHT() {
<<<<<<< HEAD
        motorLB.setPower(fPower);
        motorLF.setPower(fPower);
        motorRB.setPower(fPower);
        motorRF.setPower(fPower);
        motorLB.setTargetPosition(-fposition);
=======
        moterLB.setPower(fPower);
        motorLF.setPower(fPower);
        motorRB.setPower(fPower);
        motorRF.setPower(fPower);
        moterLB.setTargetPosition(-fposition);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setTargetPosition(-fposition);
        motorRB.setTargetPosition(-fposition);
        motorRF.setTargetPosition(-fposition);
        RUN_TO_POSITION();
        TELEMETRY2();
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void RUN_TO_POSITION() {
<<<<<<< HEAD
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (motorLB.isBusy()) {
=======
        moterLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (moterLB.isBusy()) {
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
            sleep(10);
        }
    }

    /**
     * Describe this function...
     */
    private void TURN_LEFT() {
<<<<<<< HEAD
        motorLB.setPower(fPower);
        motorLF.setPower(fPower);
        motorRB.setPower(fPower);
        motorRF.setPower(fPower);
        motorLB.setTargetPosition(fposition);
=======
        moterLB.setPower(fPower);
        motorLF.setPower(fPower);
        motorRB.setPower(fPower);
        motorRF.setPower(fPower);
        moterLB.setTargetPosition(fposition);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setTargetPosition(fposition);
        motorRB.setTargetPosition(fposition);
        motorRF.setTargetPosition(fposition);
        RUN_TO_POSITION();
        TELEMETRY2();
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void SHIFT_SIDEWAYS() {
<<<<<<< HEAD
        motorLB.setPower(-fPower);
        motorLF.setPower(fPower);
        motorRB.setPower(-fPower);
        motorRF.setPower(fPower);
        motorLB.setTargetPosition(-fposition);
=======
        moterLB.setPower(-fPower);
        motorLF.setPower(fPower);
        motorRB.setPower(-fPower);
        motorRF.setPower(fPower);
        moterLB.setTargetPosition(-fposition);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setTargetPosition(fposition);
        motorRB.setTargetPosition(-fposition);
        motorRF.setTargetPosition(fposition);
        RUN_TO_POSITION();
        TELEMETRY2();
        telemetry.update();
    }

    /**
     * Target postion for the motors
     */
    private void MOVE_FORWARD_USING_DISTANCE2() {
        double distance;

        fPower = 0.4;
        distance = 0;
<<<<<<< HEAD
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setPower(-fPower);
=======
        moterLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moterLB.setPower(-fPower);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setPower(-fPower);
        motorRB.setPower(fPower);
        motorRF.setPower(fPower);
        distance = ((DistanceSensor) color_REV_ColorRangeSensor).getDistance(DistanceUnit.MM);
        while (distance > 50) {
            distance = ((DistanceSensor) color_REV_ColorRangeSensor).getDistance(DistanceUnit.MM);
            telemetry.addData("distance", distance);
            telemetry.update();
        }
        fPower = 0;
<<<<<<< HEAD
        motorLB.setPower(-fPower);
=======
        moterLB.setPower(-fPower);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorLF.setPower(-fPower);
        motorRB.setPower(-fPower);
        motorRF.setPower(-fPower);
    }

    /**
     * Describe this function...
     */
    private void CLAW_CLOSE() {
        servoclaw.setDirection(Servo.Direction.REVERSE);
        servoclaw.setPosition(0);
        TELEMETRY2();
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void CLAW_OPEN() {
        servoclaw.setDirection(Servo.Direction.REVERSE);
        servoclaw.setPosition(0.15);
        TELEMETRY2();
        telemetry.update();
    }

    /**
     * Target postion for the motors
     */
    private void MOVE_DIAGONAL_RIGHT_BACK() {
<<<<<<< HEAD
        motorLB.setPower(3 * fPower);
=======
        moterLB.setPower(3 * fPower);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorRF.setPower(-(3 * fPower));
        motorLF.setPower(1 * fPower);
        motorRB.setPower(fPower);
        motorRB.setTargetPosition(fposition);
<<<<<<< HEAD
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
=======
        moterLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
>>>>>>> 814fb3021175d689774fa9fe176076e1f8626d97
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TELEMETRY2();
        telemetry.update();
    }
}
