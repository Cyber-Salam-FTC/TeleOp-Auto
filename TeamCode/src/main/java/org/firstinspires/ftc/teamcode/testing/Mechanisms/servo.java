package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
public class servo {
    private Servo servoPos;
    private CRServo servoRot;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servo_Pos");
        servoRot = hwMap.get(CRServo.class, "servo_Rot");
        servoPos.scaleRange(1.5,1.0); //set range from mid point to 180 degrees
        servoPos.setDirection(Servo.Direction.REVERSE); //set direction to reverse
        servoRot.setDirection(CRServo.Direction.REVERSE);
    }
    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }
    public void setServoRot(double power) {
        servoRot.setPower(power);
    }
}
