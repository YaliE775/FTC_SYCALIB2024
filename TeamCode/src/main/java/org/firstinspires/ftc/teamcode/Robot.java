package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.util.control.PIDGains;
import org.firstinspires.ftc.teamcode.util.control.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.hardware.MotorSL;
import org.firstinspires.ftc.teamcode.util.hardware.ServoSL;

public class Robot {
    public MotorSL exampleMotor;
    public MotorSL leftMotor;
    public MotorSL rightMotor;
    public MotorSL elevation;
    public ServoSL rightServo;
    public ServoSL leftServo;
    public ServoSL droneServo;
    public ServoSL intakeRight;
    public ServoSL intakeLeft;
    public ServoSL climberRight;
    public ServoSL climberLeft;

    public Motor RR;
    public Motor RF;
    public Motor LF;
    public Motor LR;
    public IMU imu;
    private static volatile Robot instance = null;
    public static Robot getInstance() {
        if(instance == null) {
            synchronized (Robot.class) {
                if(instance == null) {
                    instance = new Robot();
                }

            }
        }
        return instance;

    }


    public void init(HardwareMap hardwareMap) {

//        exampleMotor = new MotorSL(hardwareMap, "Test");
//        exampleMotor.setControlMode(MotorSL.ControlMode.POSITION);
//        exampleMotor.setPIDGains(Constants.MotorPIDGainsExample.EXAMPLE_MOTOR_PID);
//        exampleMotor.setTolerance(3);
//        exampleMotor.isUsingMotionProfiling(true);
//        exampleMotor.setProfileConstraints(new ProfileConstraints(40, 8, 8));


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.LOGO_FACING_DIR, org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        imu.resetYaw();

        PIDGains jointGains = new PIDGains(Constants.ElevatorJointPIDValues.kP, Constants.ElevatorJointPIDValues.kI, Constants.ElevatorJointPIDValues.kD);
        rightMotor = new MotorSL(hardwareMap, Constants.MotorNames.ELEVATOR_RIGHT);
        rightMotor.setControlMode(MotorSL.ControlMode.POSITION);
        rightMotor.isUsingMotionProfiling(true);
        rightMotor.useExternalSensor(true);
        rightMotor.setPIDGains(jointGains);
        rightMotor.setProfileConstraints(new ProfileConstraints(Constants.ElevatorJointPIDValues.MAX_VELOCITY, Constants.ElevatorJointPIDValues.ACCELERATION, Constants.ElevatorJointPIDValues.DECELERATION));
        rightMotor.setTolerance(Constants.ElevatorJointPIDValues.TOLERANCE);
        rightMotor.setTicksPerRev(960);
        rightMotor.setStartingAngle(-20);
        rightMotor.setFeedForward(MotorSL.FeedForwardType.COS, Constants.ElevatorJointPIDValues.kF);

        leftMotor = new MotorSL(hardwareMap, Constants.MotorNames.ELEVATOR_LEFT);
        leftMotor.setControlMode(MotorSL.ControlMode.POWER);
        leftMotor.setInverted(true);

        PIDGains elevationGains = new PIDGains(Constants.ElevatorPIDValues.kP, Constants.ElevatorPIDValues.kI, Constants.ElevatorPIDValues.kD);
        elevation = new MotorSL(hardwareMap, Constants.MotorNames.Elevation);
        elevation.setControlMode(MotorSL.ControlMode.POSITION);
        elevation.isUsingMotionProfiling(true);
        elevation.setPIDGains(elevationGains);
        elevation.setProfileConstraints(new ProfileConstraints(Constants.ElevatorPIDValues.MAX_VELOCITY, Constants.ElevatorPIDValues.ACCELERATION, Constants.ElevatorPIDValues.DECELERATION));
        elevation.setTolerance(Constants.ElevatorPIDValues.TOLERANCE);
        elevation.setInverted(true);

        leftServo = new ServoSL(hardwareMap, Constants.ServoNames.LEFT_ANGLE);
        rightServo = new ServoSL(hardwareMap, Constants.ServoNames.RIGHT_ANGLE);
        leftServo.setInverted(true);

        droneServo = new ServoSL(hardwareMap, Constants.ServoNames.DRONE);

        intakeRight = new ServoSL(hardwareMap, Constants.ServoNames.INTAKE_RIGHT);
        intakeLeft = new ServoSL(hardwareMap, Constants.ServoNames.INTAKE_LEFT);
        intakeLeft.setInverted(true);

        climberRight = new ServoSL(hardwareMap, Constants.ServoNames.RIGHT_CLIMBER);
        climberLeft = new ServoSL(hardwareMap, Constants.ServoNames.LEFT_CLIMBER);
        climberLeft.setInverted(true);

        RR = new Motor(hardwareMap, "RR");
        RF = new Motor(hardwareMap, "RF");
        RR.setInverted(true);
        RF.setInverted(true);
        RR.encoder.setDirection(Motor.Direction.REVERSE);
        RF.encoder.setDirection(Motor.Direction.REVERSE);
        RR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        LF = new Motor(hardwareMap, "LF");
        LR = new Motor(hardwareMap, "LR");
        LF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


    }

}
