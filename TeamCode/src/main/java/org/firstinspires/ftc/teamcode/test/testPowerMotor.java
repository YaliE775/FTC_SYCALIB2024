package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.test.TestMotorPIDTestSubsystem;
import org.firstinspires.ftc.teamcode.util.opmodes.TeleopBase;

@TeleOp
public class testPowerMotor extends TeleopBase {
    private TestMotorPIDTestSubsystem subsystem;
    private Robot robot;
    @Override
    public void configureBindings() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = Robot.getInstance();
        robot.init(hardwareMap);

        subsystem = new TestMotorPIDTestSubsystem(hardwareMap, telemetry);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(subsystem.setSetPoint(300));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(subsystem.setSetPoint(0));

    }
}
