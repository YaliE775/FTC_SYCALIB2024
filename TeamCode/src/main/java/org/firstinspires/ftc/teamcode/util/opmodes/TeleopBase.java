package org.firstinspires.ftc.teamcode.util.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.hardware.MotorSL;

import java.util.ArrayList;

public abstract class TeleopBase extends CommandOpMode {


    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;
    public static ArrayList<MotorSL> motors = new ArrayList<>();

    @Override
    public void initialize() {
        motors = new ArrayList<>();
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        configureBindings();
    }
    @Override
    public void run() {
        for (MotorSL motor: motors) {
            motor.periodic();
        }
        telemetry.update();
        super.run();
    }
    public abstract void configureBindings();


}
