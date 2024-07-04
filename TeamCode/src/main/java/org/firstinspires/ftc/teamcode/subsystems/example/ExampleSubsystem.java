//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.util.control.PIDGains;
//import org.firstinspires.ftc.teamcode.util.control.ProfileConstraints;
//import org.firstinspires.ftc.teamcode.util.hardware.MotorSL;
//
//public class ExampleSubsystem extends SubsystemBase {
//    Robot robot;
//    MotorSL motor;
//
//    public ExampleSubsystem() {
//        robot = Robot.getInstance();
//        motor = robot.exampleMotor;
//
//    }
//
//    public void setAngle(double setPoint) {
//        motor.setPosition(setPoint);
//    }
//
//    public boolean isAtSetPoint() {
//        return motor.isAtSetPoint();
//    }
//
//}
