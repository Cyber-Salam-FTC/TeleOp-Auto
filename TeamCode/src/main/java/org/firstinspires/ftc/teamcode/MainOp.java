    package org.firstinspires.ftc.teamcode;

    import androidx.annotation.NonNull;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;

    import org.firstinspires.ftc.teamcode.cybersalam.hardware.MecanumDrive;
    import org.firstinspires.ftc.teamcode.cybersalam.hardware.RobotHardware;

    @TeleOp(name = "Cyber Salam TeleOp")
    public class MainOp extends OpMode {

        RobotHardware hardware = new RobotHardware();
        private DcMotor leftFront;
        private DcMotor leftRear;
        private DcMotor rightFront;
        private DcMotor rightRear;

        private enum RobotState {
            RIGHT_AND_FORWARD,
            LEFT_AND_FORWARD,
            RIGHT_AND_BACKWARD,
            LEFT_AND_BACKWARD,
            STOPPED
        };

        double forward, strafe, rotate, backward;

        MecanumDrive drive = new MecanumDrive();

        @Override
        public void init() {
    //        MecanumDrive init
            drive.init(hardwareMap);




            // This is a good place to put hardware-specific logic.
            hardware.init(hardwareMap);

            // It is a good practice to also get motor objects directly
            // to make sure they are properly mapped.
            try {
                leftFront = hardwareMap.get(DcMotor.class, "leftFront");
                leftRear = hardwareMap.get(DcMotor.class, "leftRear");
                rightFront = hardwareMap.get(DcMotor.class, "rightFront");
                rightRear = hardwareMap.get(DcMotor.class, "rightRear");

                // You might need to reverse one side of the motors so your robot drives straight.
                // For example, if the right side motors spin the wrong way, you can uncomment these lines:
                // rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                // rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

            } catch (Exception e) {
                // This will tell you if there's a hardware configuration error!
                telemetry.addData("Error", "Motor initialization failed. Check your hardware map names!");
                telemetry.update();
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
            telemetry.update();
        }

    }
