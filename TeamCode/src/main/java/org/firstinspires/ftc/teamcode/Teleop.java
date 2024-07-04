package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.herev.elevator.SetElevatorHeight;
import org.firstinspires.ftc.teamcode.commands.herev.elevatorjoint.SetElevatorJointPosition;
import org.firstinspires.ftc.teamcode.subsystems.herev.*;
import org.firstinspires.ftc.teamcode.util.opmodes.TeleopBase;

@TeleOp
public class Teleop extends TeleopBase {
    private final Robot robot = Robot.getInstance();
    @Override
    public void configureBindings() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);

        ElevatorJoint elevatorJoint = new ElevatorJoint(telemetry);
        Elevator elevator = new Elevator(telemetry);
        ClawJoint clawJoint = new ClawJoint(telemetry);
        Claw claw = new Claw(telemetry);
        Drone drone = new Drone(telemetry);
        Climber climber = new Climber(telemetry);
        DriveTrain driveTrain = new DriveTrain(telemetry);
        SuperstructureTeleop superStructure = new SuperstructureTeleop(
                claw,
                clawJoint,
                elevator,
                elevatorJoint,
                telemetry
        );


        driveTrain.setDefaultCommand(
                driveTrain.fieldCentricDrive(
                () -> -gamepadEx1.getLeftX(),
                () -> -gamepadEx1.getLeftY(),
                () -> -gamepadEx1.getRightX())
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(drone.launchDrone());
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenHeld(superStructure.intakePosition(), false);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenHeld(superStructure.intakeReturn(), false);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenHeld(superStructure.scoreFirstPosition(), false);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenHeld(superStructure.scoreSecondPosition(), false);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(claw.leftToggleCommand());
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(claw.rightToggleCommand());
        gamepadEx2.getGamepadButton(GamepadKeys.Button.START).whenPressed(superStructure.goIntake());

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveOnce(new SetElevatorHeight(elevator, Constants.ElevatorHeight.HIGH, Globals.ElevatorMode.HIGH), true);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveOnce(new SetElevatorHeight(elevator, Constants.ElevatorHeight.ZERO, Globals.ElevatorMode.ZERO), true);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileActiveOnce(new SetElevatorHeight(elevator, Constants.ElevatorHeight.MIDDLE, Globals.ElevatorMode.MIDDLE), true);



    }
}
