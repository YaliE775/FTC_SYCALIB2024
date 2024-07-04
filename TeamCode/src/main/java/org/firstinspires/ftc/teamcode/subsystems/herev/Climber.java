package org.firstinspires.ftc.teamcode.subsystems.herev;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.hardware.ServoSL;

public class Climber extends SubsystemBase {

    ServoSL climberRight;
    ServoSL climberLeft;
    private final Robot robot = Robot.getInstance();
    private final Globals globals = Globals.getInstance();
    public Climber(Telemetry telemetry) {
        climberRight = robot.climberRight;
        climberLeft = robot.climberLeft;
    }

    public Command climberUp() {
        return new InstantCommand(() -> climberRight.setPosition(Constants.ClimberPosition.OPEN))
                .alongWith(new InstantCommand(() -> climberLeft.setPosition(Constants.ClimberPosition.OPEN)));
    }

}

