package org.firstinspires.ftc.teamcode.subsystems.herev;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.hardware.MotorSL;

public class Elevator extends SubsystemBase {

    private MotorSL elevation;
    private final Robot robot = Robot.getInstance();
    private final Globals globals = Globals.getInstance();
    private double currentSetPoint;
    Telemetry telemetry;
    public Elevator(Telemetry telemetry) {
        this.telemetry = telemetry;
        elevation = robot.elevation;
    }

    public void setSetPoint(double setPoint) {
        elevation.setPosition(setPoint);
    }

    public boolean isAtSetPoint() {
        return elevation.isAtSetPoint() && elevation.getSetPoint() == currentSetPoint;
    }

    @Override
    public void periodic() {
        telemetry.addData("E State:", globals.elevatorMode);
        telemetry.addData("E Position:", elevation.getPosition());
        telemetry.addData("E Set Point :", elevation.getSetPoint());
        telemetry.addData("E Velocity:", elevation.getCurrentMotionState().v);
    }
}
