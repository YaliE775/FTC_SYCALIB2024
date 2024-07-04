package org.firstinspires.ftc.teamcode.util.hardware;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.control.*;
import org.firstinspires.ftc.teamcode.util.opmodes.TeleopBase;

public class MotorSL {
    public enum ControlMode {
        POWER,
        POSITION,
        VELOCITY;
    }

    public enum FeedForwardType {
        NONE,
        CONSTANT,
        COS,
        SIN;
    }
    private ControlMode mode;

    private boolean isUsingMotionProfiling;
    private double kF;
    private AsymmetricMotionProfile motionProfile;
    private MotionState currentMotionState;
    private ProfileConstraints profileConstraints;
    private double currentSetPoint;
    private double sensorValue;
    private boolean isUsingExternalSensor;
    ElapsedTime time;

    private PIDControllerS positionControl;
    private PIDControllerS velocityControl;
    private FeedForwardType feedForwardType;

    private double ticksPerRev;
    private double startingAngle;
    private double tprMultiplier;

//    private MotorSL leadMotor;
    private DcMotorEx motor;

    public MotorSL(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        positionControl = new PIDControllerS(new PIDGains(0,0,0));
        mode = ControlMode.POWER;
        isUsingMotionProfiling = false;
        feedForwardType = FeedForwardType.NONE;
        time = new ElapsedTime();
        profileConstraints = new ProfileConstraints(1,1,1);
        motionProfile = new AsymmetricMotionProfile(motor.getCurrentPosition(), motor.getCurrentPosition(), profileConstraints);
        currentMotionState = new MotionState(0,0,0);
        startingAngle = 0;
        isUsingExternalSensor = false;


        TeleopBase.motors.add(this);
    }

    public void setControlMode(ControlMode mode) {
        this.mode = mode;

    }

    public void setPower(double power) {
        if(mode != ControlMode.POWER) return;
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setInverted(boolean setInverted) {
        if(setInverted) motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPIDGains(PIDGains pidGains) {
        positionControl.setPID(pidGains.getProportional(), pidGains.getIntegral(), pidGains.getDerivative());
    }

    public void isUsingMotionProfiling(boolean isUsingMotionProfiling) {
        this.isUsingMotionProfiling = isUsingMotionProfiling;
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public void setPosition(double setPoint) {
        currentSetPoint = setPoint;
        time = new ElapsedTime();
        positionControl.setSetPoint(setPoint);
        if(isUsingMotionProfiling) {
            motionProfile = new AsymmetricMotionProfile(getSensorValue(), setPoint, profileConstraints);
        }
    }

    public void setTolerance(double tolerance) {positionControl.setTolerance(tolerance);}

    public void setTicksPerRev(double ticksPerRev) {
        this.ticksPerRev = ticksPerRev;
        tprMultiplier = ticksPerRev / 360;
    }

    public void setStartingAngle(double angle) {startingAngle = angle;}

    public double getMotorAngle() {
        return motor.getCurrentPosition() / tprMultiplier + startingAngle;
    }

    public double getPIDPower(double currentTime) {
        if(mode == ControlMode.VELOCITY) {

        }
        if(isUsingMotionProfiling) {
            currentMotionState = motionProfile.getMotionState(currentTime);
            return positionControl.calculate(getSensorValue(), currentMotionState.x);
        }

        return positionControl.calculate(motor.getCurrentPosition());
    }

    public void setFeedForward(FeedForwardType feedForwardType, double kF) {
        this.feedForwardType = feedForwardType;
        this.kF = kF;
    }

    public void setVelocity(double setVelocity) {
        velocityControl.setSetPoint(setVelocity);
    }

    public void setVelocityTolerance(double tolerance) {
        velocityControl.setTolerance(tolerance);
    }

//    public void setPIDGains(double tolerance) {
//        velocityControl.setTolerance(tolerance);
//    }

    public double getFeedForwardPower() {
        switch (feedForwardType) {
            case CONSTANT:
                return kF;
            case COS:
                return Math.cos(Math.toRadians(getMotorAngle())) * kF;
            case SIN:
                return Math.sin(Math.toRadians(getMotorAngle())) * kF;
            default:
                return 0;
        }
    }
    public void setProfileConstraints(ProfileConstraints profileConstraints) {
        this.profileConstraints = profileConstraints;
    }

    public boolean isAtSetPoint() {
        return positionControl.atSetPoint();
    }
    public double getSetPoint() {
        return positionControl.getSetPoint();
    }

    public MotionState getCurrentMotionState() {
        return currentMotionState;
    }

    public double getCurrentTime() {
        return time.seconds();
    }
    public double getTprMultiplier() {
        return tprMultiplier;
    }
    public void useExternalSensor(boolean isUsingExternalSensor) {
        this.isUsingExternalSensor = isUsingExternalSensor;
    }

    public void setSensorValue(double value) {
        sensorValue = value;
    }

    public double getSensorValue() {
        return sensorValue;
    }



//    public void follow(MotorSL leadMotor) {
//        if(mode != ControlMode.FOLLOWER) return;
//        this.leadMotor = leadMotor;
//    }

    public void periodic() {
        if(mode == ControlMode.POSITION) {
            if(!isUsingExternalSensor) setSensorValue(getPosition());
            double power = getPIDPower(time.seconds());
            if(positionControl.atSetPoint() && positionControl.getSetPoint() == currentSetPoint) power = 0;
            motor.setPower(power + getFeedForwardPower());
        } else {

        }

    }

}
