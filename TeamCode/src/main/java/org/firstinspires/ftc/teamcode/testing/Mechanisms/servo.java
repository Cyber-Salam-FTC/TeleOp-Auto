package org.firstinspires.ftc.teamcode.testing.Mechanisms;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
public class servo {
    private Servo servoPos;
    private CRServo servoRot;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servo_Pos");
        servoRot = hwMap.get(CRServo.class, "servo_Rot");
//        servoPos.scaleRange(0, 1);
        servoPos.setDirection(Servo.Direction.REVERSE); //set direction to reverse
        servoPos.setPosition(0);
    }
    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }
    public void setServoRot(double power) {
        servoRot.setPower(power);
    }
}
