package org.firstinspires.ftc.teamcode.commands.herev.elevatorjoint;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.subsystems.herev.ElevatorJoint;

public class SetElevatorJointPosition extends CommandBase {
    private ElevatorJoint elevatorJoint;
    private double setPoint;
    private final Globals globals = Globals.getInstance();
    Globals.ElevatorJointMode elevatorJointMode;
    public SetElevatorJointPosition(ElevatorJoint elevatorJoint, double setPoint, Globals.ElevatorJointMode elevatorJointMode) {
        this.elevatorJointMode = elevatorJointMode;
        this.elevatorJoint = elevatorJoint;
        this.setPoint = setPoint;
    }

    @Override
    public void initialize() {
        elevatorJoint.setSetPoint(setPoint);
        globals.elevatorJointMode = elevatorJointMode;
    }


    @Override
    public boolean isFinished() {
        return elevatorJoint.isAtSetPoint();
    }
}
