package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name = "CyberSalam - TeleOp")

public class Main extends LinearOpMode {

    private DcMotor intake;
    private DcMotor armtop;
    private DcMotor armbottom;
    private DcMotor moterLB;
    private DcMotor motorLF;
    private DcMotor motorRB;
    private DcMotor motorRF;
    private Servo servoclaw;
    public CRServo wedge;

    int limit_reached_armtop = 0;
    int limit_reached_armbottom = 0;
    float ultraPower = 0;
    double fIntake = 0;
    float fArmTop = 0;
    float bPower = 0;
    float switch1 = 0;
    double fPower = 0;
    double sPower = 0;
    double dPower = 0;
    int armbottomposition = 0;
    int armtopposition = 0;

    @Override
    public void runOpMode() {

        moterLB = hardwareMap.get(DcMotor.class, "moterLB");
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");
        armtop = hardwareMap.get(DcMotor.class, "armtop");
        armbottom = hardwareMap.get(DcMotor.class, "armbottom");
        servoclaw = hardwareMap.get(Servo.class, "servoclaw");
        wedge = hardwareMap.get(CRServo.class, "wedge");
        intake = hardwareMap.get(DcMotor.class, "intake");
//        wedge = hardwareMap.get(Servo.class, "wedge");

        // Makes robot brake when controller isn't giving any input.
        moterLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armtop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armbottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders.
        MOTORS_STOP_AND_RESET_ENCODERS();
        MOTORS_RUN_WITHOUT_ENCODERS();

        telemetry.addData("Status", "Initialize ... DONE!");
        telemetry.update();

        // Put initialization blocks here.
        servoclaw.setDirection(Servo.Direction.REVERSE);
        servoclaw.setPosition(0.0);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
//                Example to diagonally turn
                if (0 < gamepad2.left_stick_y) {
                    fIntake = -(gamepad2.left_stick_y) * 0.5;
                    intake.setPower(fIntake);
                } else if (0 > gamepad2.left_stick_y) {
                    fIntake = -(gamepad2.left_stick_y) * 0.5;
                    intake.setPower(fIntake);
                } else if (0 == gamepad2.left_stick_y) {
                    fIntake = 0;
                    intake.setPower(fIntake);
                }

                if (gamepad2.a) {
                    wedge.setPower(1);
                } else if (gamepad2.b){
                    wedge.setPower(-1);
                } else {
                    wedge.setPower(0);
                }

//                if (gamepad2.b) {
//                    switch1 = 1;
//                }
//
//                if (gamepad2.a){
//                    switch1 = 2;
//                }
//
//                if (switch1 == 1) {
//                    wedge.
//                }
//
//                if (switch1 == 2) {
//                    if (gamepad2.left_stick_x > 0){
//                        wedge.setPower(0.3);
//                    } else if (gamepad2.left_stick_x < 0) {
//                        wedge.setPower(-0.3);
//                    } else if (gamepad2.left_stick_x == 0) {
//                        wedge.setPower(0);
//                    }
//                }



                // Put loop blocks here.


//                if (gamepad2.dpad_up){
//                    INTAKE_OPEN();
//                }
//
//                if (gamepad2.dpad_down){
//                    INTAKE_CLOSE();
//                }

                SOFTWARE_LIMIT();

                float topStop = 0.3F;
                if ( (gamepad2.right_stick_y > 0) && (limit_reached_armtop == 0)) {
                    fArmTop = -(gamepad2.right_stick_y);
                    armtop.setPower(fArmTop);
                }  else if (gamepad2.right_stick_y < 0) {
                    fArmTop = -gamepad2.right_stick_y;
                    armtop.setPower(fArmTop);
                } else {
                    fArmTop = 0;
                    armtop.setPower(0);
                }

                if (0 != gamepad2.right_trigger && limit_reached_armbottom == 0 ) {
                    ultraPower = gamepad2.right_trigger;
                    armbottom.setPower(ultraPower);
                } else if (0 != gamepad2.left_trigger) {
                    ultraPower = -gamepad2.left_trigger;
                    armbottom.setPower(ultraPower);
                } else {
                    armbottom.setPower(0);
                    ultraPower = 0;
                }





                if (gamepad2.right_bumper) {
                    CLAW_OPEN();
                }
                if (gamepad2.left_bumper) {
                    CLAW_CLOSE();
                }




//                // PS/Home button: Reset robot configuration
//                if (gamepad2.ps) {
//                    ultraPower = (float) 0.8;
//                    armtopposition = 0;
//                    ARM_TOP();
//                    while ( armtop.isBusy() ) {
//                        sleep(10);
//                    }
//                    armbottomposition = 0;
//                    ARM_BOTTOM();
//                    while ( armbottom.isBusy() ) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }
//                // L1 : Perimeter Position
//                if (gamepad2.left_bumper) {
//                    ultraPower = (float) 0.8;
//                    armbottomposition = 1400;
//                    ARM_BOTTOM();
//                    while (armbottom.isBusy()) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }
//                // R1: Specimen hanging position
//                if (gamepad2.right_bumper) {
//                    ultraPower = (float) 0.8;
//                    armbottomposition = 3200;
//                    ARM_BOTTOM();
//                    while (armbottom.isBusy()) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }
//                if (gamepad2.circle) {
//                    CLAW_OPEN();
//                }
//                if (gamepad2.cross) {
//                    CLAW_CLOSE();
//                }
//                // Square : Climb position (90 degrees)
//                if (gamepad2.square) {
//                    ultraPower = (float) 0.8;
//                    armbottomposition = 2700;
//                    ARM_BOTTOM();
//                    while (armbottom.isBusy()) {
//                        sleep(10);
//                    }
//                    armtopposition = 350;
//                    ARM_TOP();
//                    while (armtop.isBusy()) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }
//                // Triangle/Y: Bucket position
//                if (gamepad2.triangle) {
//                    ultraPower = (float) 0.8;
//                    armbottomposition = 2750;
//                    ARM_BOTTOM();
//                    while (armbottom.isBusy()) {
//                        sleep(10);
//                    }
//                    armtopposition = 350;
//                    ARM_TOP();
//                    while (armtop.isBusy()) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }

                telemetry.addData("armbottom", armbottom.getCurrentPosition());
                telemetry.addData("armtop", armtop.getCurrentPosition());
                telemetry.addData("limit_reachedarmtop", limit_reached_armtop);
                telemetry.addData("limit_reachedarmbottom", limit_reached_armbottom);
                telemetry.addData("F1", fPower);
                telemetry.addData("B1", bPower);
                telemetry.addData("S1", sPower);
                telemetry.addData("D1", dPower);
                telemetry.addData("ultraPower", ultraPower);
                telemetry.addData("Intake", fIntake);
                telemetry.addData("Claw", servoclaw.getPosition());
                telemetry.addData("gamepad2.right_trigger", gamepad2.right_trigger);
//                telemetry.addData("Intake Claw", wedge.getPosition());
                telemetry.addData("armbottom_zeroPower", armbottom.getZeroPowerBehavior());
                telemetry.addData("wedgePower", wedge.getPower());
                telemetry.addData("left_stick_x", gamepad2.left_stick_x);
                telemetry.addData("armtop_zeroPower", armtop.getZeroPowerBehavior());
                telemetry.update();
            }

            // Put run blocks here.

        }

    }

    private void MOTORS_STOP_AND_RESET_ENCODERS() {
        armbottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armtop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void MOTORS_RUN_WITHOUT_ENCODERS() {
        armbottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armtop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Describe this function...
     */

    private void SOFTWARE_LIMIT() {
        if (armbottom.getCurrentPosition() > 3900) {
            limit_reached_armbottom = 1;
            if (armtop.getCurrentPosition() > 280) {
                limit_reached_armtop = 1;
            } else {
                limit_reached_armtop = 0;
            }
        } else {
            limit_reached_armbottom = 0;
            limit_reached_armtop = 0;
        }
//        if (armtop.getCurrentPosition() > 280) {
//            limit_reached_armtop = 1;
//        } else {
//            limit_reached_armtop = 0;
//        }
    }

    /**
     * Describe this function...
     */
    private void ARM_BOTTOM() {
        armbottom.setPower((4 * ultraPower));
        armbottom.setTargetPosition(armbottomposition);
        armbottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void ARM_TOP() {
        armtop.setPower((3 * ultraPower));
        armtop.setTargetPosition(armtopposition);
        armtop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void CLAW_OPEN() {
        servoclaw.setDirection(Servo.Direction.REVERSE);
        servoclaw.setPosition(0.15);
    }
//
//    private void INTAKE_OPEN() {
////        wedge.setDirection(Servo.Direction.REVERSE);
//        wedge.setPosition(0.4);
//    }

    /**
     * Describe this function...
     */
    private void CLAW_CLOSE() {
        servoclaw.setDirection(Servo.Direction.REVERSE);
        servoclaw.setPosition(0);
    }

//    private void INTAKE_CLOSE() {
////        wedge.setDirection(Servo.Direction.REVERSE);
//        wedge.setPosition(0);
//    }
}