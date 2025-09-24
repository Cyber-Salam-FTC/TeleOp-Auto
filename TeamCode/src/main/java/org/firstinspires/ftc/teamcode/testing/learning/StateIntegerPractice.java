package org.firstinspires.ftc.teamcode.testing.learning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cybersalam.hardware.RobotHardware;

public class StateIntegerPractice extends OpMode {

    RobotHardware hardware = new RobotHardware();
    int state;

    enum State {
        WAIT_FOR_A,
        WAIT_FOR_B,
        WAIT_FOR_X,
        FINSIHED
    }

    State curstate = State.WAIT_FOR_A;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        state = 0;
        curstate = State.WAIT_FOR_A;
    }

    @Override
    public void loop() {
//        Number based
        telemetry.addData("Current State", state);

        switch (state) {
            case 0:
                telemetry.addLine("To exit state press A");
                if (gamepad1.a) {
                    state = 1;
                }
                break;
            case 1:
                telemetry.addLine("To exit state press B");
                if (gamepad1.b) {
                    state = 2;
                }
                break;
            case 2:
                telemetry.addLine("To exit state press X");
                if (gamepad1.x) {
                    state = 3;
                }
                break;
            default:
                telemetry.addLine("Auto State machine completed");
        }

//        Text based (using Enum)

        switch (curstate) {
            case WAIT_FOR_A:
                telemetry.addLine("To exit state press A");
                if (gamepad1.a) {
                    curstate = State.WAIT_FOR_B;
                }
                break;
            case WAIT_FOR_B:
                telemetry.addLine("To exit state press B");
                if (gamepad1.b) {
                    curstate = State.WAIT_FOR_X;
                }
                break;
            case WAIT_FOR_X:
                telemetry.addLine("To exit state press X");
                if (gamepad1.x) {
                    curstate = State.FINSIHED;
                }
                break;
            default:
                telemetry.addLine("Auto State machine completed");
        }
    }
}
