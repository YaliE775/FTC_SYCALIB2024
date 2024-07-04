package org.firstinspires.ftc.teamcode.subsystems.test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.hardware.MotorSL;

public class TestMotorPowerSubsystem extends SubsystemBase {
    MotorSL motorSL;
    Telemetry telemetry;
    public TestMotorPowerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        motorSL = new MotorSL(hardwareMap, "Test");
//        motorSL.setControlMode(MotorSL.ControlMode.POWER);
        this.telemetry = telemetry;
    }

    public Command setPower(double power) {
        return new InstantCommand(() -> motorSL.setPower(power));
    }
    public double getPower() {return motorSL.getPower();}

    @Override
    public void periodic() {
        telemetry.addData("bolbol", motorSL.getPower());
        telemetry.update();
    }
}
