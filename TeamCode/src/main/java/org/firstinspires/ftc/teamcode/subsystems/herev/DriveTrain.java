package org.firstinspires.ftc.teamcode.subsystems.herev;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    public Motor RR;
    public Motor RF;
    public Motor LF;
    public Motor LR;
    private MecanumDrive drive;
    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;
    private MecanumDriveWheelSpeeds wheelSpeeds;
    Translation2d LFLocation = new Translation2d(0.1575,0.15);
    Translation2d RFLocation = new Translation2d(0.1575,-0.15);
    Translation2d LRLocation = new Translation2d(-0.1575,0.15);
    Translation2d RRLocation = new Translation2d(-0.1575,-0.15);
    IMU imu;
    Rotation2d heading;
    Telemetry telemetry;
    ElapsedTime timer;
    double x;
    public DriveTrain(Telemetry telemetry) {
        RR = robot.RR;
        RF = robot.RF;
        LF = robot.LF;
        LR = robot.LR;

        drive = new MecanumDrive(LF, RF, LR, RR);
        drive.setRightSideInverted(false);
        drive.setMaxSpeed(1);

        imu = robot.imu;
        heading = Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        kinematics = new MecanumDriveKinematics(LFLocation, RFLocation, LRLocation, RRLocation);
        odometry = new MecanumDriveOdometry(kinematics, heading);

        timer = new ElapsedTime();
        this.telemetry = telemetry;
    }

    public Command robotCentricDrive(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        return new RunCommand(() -> drive.driveRobotCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(), turnSpeed.getAsDouble()), this);
    }

    public Command fieldCentricDrive(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        x = turnSpeed.getAsDouble();
        return new RunCommand(() -> drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(), turnSpeed.getAsDouble(), robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), this);
    }

    @Override
    public void periodic() {
        wheelSpeeds = new MecanumDriveWheelSpeeds(
                LF.getRate(), RF.getRate(), LR.getRate(), RR.getRate()
        );
        heading = Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        odometry.updateWithTime(timer.seconds(), heading, wheelSpeeds);
        telemetry.addData("Robot Position:", odometry.getPoseMeters());
        telemetry.addData("right x:", x);
    }
}
