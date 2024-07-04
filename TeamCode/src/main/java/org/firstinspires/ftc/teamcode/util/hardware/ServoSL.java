package org.firstinspires.ftc.teamcode.util.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoSL {

    ServoEx servo;
    public ServoSL(HardwareMap hardwareMap, String name) {
        servo = new SimpleServo(hardwareMap, name, 0,0);
    }

    public void setPosition(double setPosition) {
        servo.setPosition(setPosition);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void setInverted(boolean isInverted) {
        servo.setInverted(isInverted);
    }
}
