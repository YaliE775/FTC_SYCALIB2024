package org.firstinspires.ftc.teamcode.subsystems.herev;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.hardware.ServoSL;

public class ClawJoint extends SubsystemBase {

    private ServoSL rightServo;
    private ServoSL leftServo;
    Telemetry telemetry;
    private final Robot robot = Robot.getInstance();
    private final Globals globals = Globals.getInstance();

    public ClawJoint(Telemetry telemetry) {
        this.telemetry = telemetry;
        leftServo = robot.leftServo;
        rightServo = robot.rightServo;
    }

    public void setAngle(double position) {
        rightServo.setPosition(position);
        leftServo.setPosition(position);
    }

    public Command intake() {
        return new InstantCommand(() -> setAngle(Constants.ServoAngle.INTAKE))
                .andThen(new InstantCommand(() -> globals.clawAngleMode = Globals.ClawAngleMode.INTAKE));
    }

    public Command scoreOne() {
        return new InstantCommand(() -> setAngle(Constants.ServoAngle.SCORE_ONE))
                .andThen(new InstantCommand(() -> globals.clawAngleMode = Globals.ClawAngleMode.SCORE_ONE));
    }

    public Command scoreTwo() {
        return new InstantCommand(() -> setAngle(Constants.ServoAngle.SCORE_TWO)).
                andThen(new InstantCommand(() -> globals.clawAngleMode = Globals.ClawAngleMode.SCORE_TWO));
    }
    public Command hold() {
        return new InstantCommand(() -> setAngle(Constants.ServoAngle.HOLD))
                .andThen(new InstantCommand(() -> globals.clawAngleMode = Globals.ClawAngleMode.HOLD));
    }

    @Override
    public void periodic() {
        telemetry.addData("Claw Angle State:", globals.clawAngleMode);
    }
}
