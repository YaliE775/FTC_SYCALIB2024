package org.firstinspires.ftc.teamcode.subsystems.herev;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.hardware.MotorSL;

public class ElevatorJoint extends SubsystemBase {
    private MotorSL leftMotor; //follower
    private MotorSL rightMotor; //lead motor
    private final Robot robot = Robot.getInstance();
    private final Globals globals = Globals.getInstance();
    private double currentSetPoint;
    Telemetry telemetry;

    public ElevatorJoint(Telemetry telemetry) {
        leftMotor = robot.leftMotor;
        rightMotor = robot.rightMotor;
        this.telemetry = telemetry;

    }

    public void setSetPoint(double setPoint) {
        rightMotor.setPosition(setPoint);
        currentSetPoint = setPoint;
    }

    public boolean isAtSetPoint() {
        return rightMotor.isAtSetPoint() && rightMotor.getSetPoint() == currentSetPoint;
    }

    @Override
    public void periodic() {
        rightMotor.setSensorValue((rightMotor.getPosition() + leftMotor.getPosition()) / 2);
        double power = rightMotor.getPIDPower(rightMotor.getCurrentTime()) + rightMotor.getFeedForwardPower();
        if(rightMotor.isAtSetPoint() && rightMotor.getSetPoint() == currentSetPoint) power = 0;
        leftMotor.setPower(power);

        telemetry.addData("Elevator Joint State:", globals.elevatorJointMode);
        telemetry.addData("EJ Position:", (rightMotor.getPosition() + leftMotor.getPosition()) / 2);
        telemetry.addData("EJ Set Point:", rightMotor.getSetPoint());
        telemetry.addData("EJ Velocity:", rightMotor.getCurrentMotionState().v);
        telemetry.addData("Angle", rightMotor.getMotorAngle());
    }
}
