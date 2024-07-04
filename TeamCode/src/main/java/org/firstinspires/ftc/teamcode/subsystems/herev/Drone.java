package org.firstinspires.ftc.teamcode.subsystems.herev;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.hardware.ServoSL;

public class Drone extends SubsystemBase {

    Telemetry telemetry;
    private ServoSL droneServo;
    private final Robot robot = Robot.getInstance();
    public Drone(Telemetry telemetry) {
        this.telemetry = telemetry;
        droneServo = robot.droneServo;
    }

    public Command launchDrone() {
        return new InstantCommand(() -> droneServo.setPosition(1));
    }


}
