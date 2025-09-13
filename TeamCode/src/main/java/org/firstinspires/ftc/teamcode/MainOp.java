package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cybersalam.hardware.RobotHardware;

@TeleOp(name = "Cyber Salam TeleOp")
public class MainOp extends OpMode {

    RobotHardware hardware = new RobotHardware();

    private enum state {
        WAIT_FOR_RIGHT_TRIGGER
    }



    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {


        switch (){
            
        }
    }

}