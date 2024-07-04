package org.firstinspires.ftc.teamcode.subsystems.herev;

import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.commands.herev.elevator.SetElevatorHeight;
import org.firstinspires.ftc.teamcode.commands.herev.elevatorjoint.SetElevatorJointPosition;

public class SuperstructureTeleop extends SubsystemBase {

    private ElevatorJoint elevatorJoint;
    private Elevator elevator;
    private ClawJoint clawJoint;
    private Claw claw;
    private Telemetry telemetry;

    private final Globals globals = Globals.getInstance();
    public SuperstructureTeleop(Claw claw, ClawJoint clawJoint, Elevator elevator, ElevatorJoint elevatorJoint, Telemetry telemetry) {
        this.claw = claw;
        this.clawJoint = clawJoint;
        this.elevator = elevator;
        this.elevatorJoint = elevatorJoint;
        this.telemetry = telemetry;
    }

    public Command intakePosition() {
        return new SequentialCommandGroup(
                clawJoint.intake(),
                new ParallelCommandGroup(
                        new WaitCommand(200),
                        claw.openLeftIntake(),
                        claw.openRightIntake(),
                        new SetElevatorHeight(elevator, Constants.ElevatorHeight.HIGH, Globals.ElevatorMode.HIGH)
                        )

        ).alongWith(new InstantCommand(() -> globals.robotMode = Globals.RobotMode.INTAKE));
    }

    public Command intakeReturn() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        claw.closeLeftIntake(),
                        claw.closeRightIntake()
                ),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new SetElevatorHeight(elevator, Constants.ElevatorHeight.ZERO, Globals.ElevatorMode.ZERO),
                        new WaitCommand(200),
                        clawJoint.hold()

                )

        ).alongWith(new InstantCommand(() -> globals.robotMode = Globals.RobotMode.GO_SCORE));
    }

    public Command scoreFirstPosition() {
        return new ParallelCommandGroup(
                clawJoint.scoreOne(),
                new SetElevatorJointPosition(elevatorJoint, Constants.ElevatorAngles.SCORE, Globals.ElevatorJointMode.SCORE)

        ).alongWith(new InstantCommand(() -> globals.robotMode = Globals.RobotMode.SCORE));
    }

    public Command scoreSecondPosition() {
        return new ParallelCommandGroup(
                clawJoint.scoreTwo(),
                new SetElevatorJointPosition(elevatorJoint, Constants.ElevatorAngles.SCORE, Globals.ElevatorJointMode.SCORE)

        ).alongWith(new InstantCommand(() -> globals.robotMode = Globals.RobotMode.SCORE));
    }

    public Command goIntake() {
        return new ParallelCommandGroup(
                new SetElevatorJointPosition(elevatorJoint, Constants.ElevatorAngles.INTAKE, Globals.ElevatorJointMode.INTAKE),
                new WaitCommand(600),
                claw.closeRightIntake(),
                claw.closeLeftIntake(),
                clawJoint.hold()

        ).alongWith(new InstantCommand(() -> globals.robotMode = Globals.RobotMode.GO_INTAKE));
    }


    @Override
    public void periodic() {
        telemetry.addData("Robot State:", globals.robotMode);
    }
}
