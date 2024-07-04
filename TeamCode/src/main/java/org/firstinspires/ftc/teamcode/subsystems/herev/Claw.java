package org.firstinspires.ftc.teamcode.subsystems.herev;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.hardware.ServoSL;

import java.util.function.BooleanSupplier;

public class Claw extends SubsystemBase {
    private ServoSL intakeRight;
    private ServoSL intakeLeft;
    Telemetry telemetry;
    private final Robot robot = Robot.getInstance();
    private final Globals globals = Globals.getInstance();
    BooleanSupplier canRun;

    public Claw(Telemetry telemetry) {
        intakeRight = robot.intakeRight;
        intakeLeft = robot.intakeLeft;
        this.telemetry = telemetry;
        canRun = () -> globals.robotMode != Globals.RobotMode.GO_SCORE && globals.robotMode != Globals.RobotMode.GO_INTAKE;

    }

    public Command closeRightIntake() {
        globals.rightIntakeMode = Globals.ClawIntakeMode.CLOSED;
        return new InstantCommand(() -> intakeRight.setPosition(Constants.IntakePositions.CLOSED)).
                andThen(new InstantCommand(() -> globals.rightIntakeMode = Globals.ClawIntakeMode.CLOSED));
    }

    public Command closeLeftIntake() {
        globals.leftIntakeMode = Globals.ClawIntakeMode.CLOSED;
        return new InstantCommand(() -> intakeLeft.setPosition(Constants.IntakePositions.CLOSED)).
                andThen(new InstantCommand(() -> globals.leftIntakeMode = Globals.ClawIntakeMode.CLOSED));
    }

    public Command openRightIntake() {
        return new InstantCommand(() -> intakeRight.setPosition(Constants.IntakePositions.OPEN)).
                andThen(new InstantCommand(() -> globals.rightIntakeMode = Globals.ClawIntakeMode.OPENED));
    }

    public Command openLeftIntake() {
        return new InstantCommand(() -> intakeLeft.setPosition(Constants.IntakePositions.OPEN)).
                andThen(new InstantCommand(() -> globals.leftIntakeMode = Globals.ClawIntakeMode.OPENED));
    }

    public Command rightToggleCommand() {

        return new ConditionalCommand(
                new ConditionalCommand(
                    openRightIntake(),
                    closeRightIntake(),
                    () -> globals.rightIntakeMode == Globals.ClawIntakeMode.CLOSED
                ),
            new InstantCommand(),
            canRun

        );


    }

    public Command leftToggleCommand() {

        return new ConditionalCommand(
                new ConditionalCommand(
                        openLeftIntake(),
                        closeLeftIntake(),
                        () -> globals.leftIntakeMode == Globals.ClawIntakeMode.CLOSED
                ),
                new InstantCommand(),
                canRun

        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Right Intake State:", globals.rightIntakeMode);
        telemetry.addData("Left Intake State:", globals.leftIntakeMode);
    }
}
