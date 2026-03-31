package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.cybersalam.hardware.MecanumDrive;

@TeleOp(name = "Main TeleOp")
public class main extends LinearOpMode {
    @Override
    public void runOpMode() {
//        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
//        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
//        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
//        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        double CLOSE_VELOCITY = 1011;
        double INTAKE_SPEED = 1;

//        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        waitForStart();

        while (opModeIsActive()) {
            MecanumDrive drive = new MecanumDrive();
//
//            drive.init(hardwareMap);
//
//            double forward = gamepad1.right_trigger - gamepad1.left_trigger;
//            double strafe = gamepad1.left_stick_x;
//            double rotate = gamepad1.right_stick_x;
//
//            drive.drive(forward, strafe, rotate);

//            if (gamepad2.dpad_left) {
////                distance from target position 100in
//                shooter.setVelocity(rpmToTps(CLOSE_VELOCITY));
//            }

            if (gamepad2.cross) {
                intake.setPower(INTAKE_SPEED);
            } else {
                intake.setPower(0);
            }

        }
    }

    public double rpmToTps(double rpm) {
        return rpm * 28;
    }

}
