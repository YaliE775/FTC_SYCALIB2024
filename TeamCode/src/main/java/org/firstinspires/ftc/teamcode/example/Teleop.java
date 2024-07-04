//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.Subsystem;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.commands.ExampleCommand;
//import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem;
//import org.firstinspires.ftc.teamcode.util.hardware.MotorSL;
//import org.firstinspires.ftc.teamcode.util.opmodes.TeleopBase;
//
//import java.util.ArrayList;
//
//@TeleOp
//public class Teleop extends TeleopBase {
//    Robot robot;
//    ExampleSubsystem exampleSubsystem;
//
//    @Override
//    public void configureBindings() {
//        robot = Robot.getInstance();
//        robot.init(hardwareMap);
//
//        exampleSubsystem = new ExampleSubsystem();
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whileActiveContinuous(new ExampleCommand(exampleSubsystem));
//    }
//
//
//}
