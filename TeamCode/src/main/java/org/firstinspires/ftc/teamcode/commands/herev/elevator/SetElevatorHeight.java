package org.firstinspires.ftc.teamcode.commands.herev.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.subsystems.herev.Elevator;

public class SetElevatorHeight extends CommandBase {

    private Elevator elevator;
    private double setPoint;
    private final Globals globals = Globals.getInstance();
    Globals.ElevatorMode elevatorMode;
    public SetElevatorHeight(Elevator elevator, double setPoint, Globals.ElevatorMode elevatorMode) {
        this.elevatorMode = elevatorMode;
        this.elevator = elevator;
        this.setPoint = setPoint;
    }

    @Override
    public void initialize() {
        elevator.setSetPoint(setPoint);
        globals.elevatorMode = elevatorMode;
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtSetPoint();
    }
}
