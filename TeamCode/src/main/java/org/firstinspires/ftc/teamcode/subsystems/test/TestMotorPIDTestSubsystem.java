package org.firstinspires.ftc.teamcode.subsystems.test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.control.PIDGains;
import org.firstinspires.ftc.teamcode.util.hardware.MotorSL;

public class TestMotorPIDTestSubsystem extends SubsystemBase {
    MotorSL motorSL;
    Robot robot;
    Telemetry telemetry;
    public TestMotorPIDTestSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        robot = Robot.getInstance();
        motorSL = robot.exampleMotor;
    }

    public Command setSetPoint(double power) {
        return new InstantCommand(() -> motorSL.setPosition(power));
    }
    public double getPower() {return motorSL.getPower();}

    @Override
    public void periodic() {
        telemetry.addData("setPoint", motorSL.getSetPoint());
        telemetry.addData("motion state x", motorSL.getCurrentMotionState().x);
        telemetry.addData("position", motorSL.getPosition());
        telemetry.addData("time", motorSL.getCurrentTime());
        telemetry.update();
    }
}
