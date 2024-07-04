package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.control.PIDGains;
import org.firstinspires.ftc.teamcode.util.control.ProfileConstraints;

public class Constants {

    public static class MotorNames {
        public static final String EXAMPLE_MOTOR = "Motor";
        public static final String ELEVATOR_RIGHT = "Elevator Right";
        public static final String ELEVATOR_LEFT = "Elevator Left";
        public static final String Elevation = "Elevation";
    }

    public static class ServoNames {
        public static final String RIGHT_ANGLE = "Right Angle";
        public static final String LEFT_ANGLE = "Left Angle";
        public static final String INTAKE_RIGHT = "Intake Right";
        public static final String INTAKE_LEFT = "Intake Left";
        public static final String LEFT_CLIMBER = "Left Climber";
        public static final String RIGHT_CLIMBER = "Right Climber";
        public static final String DRONE = "Drone";

    }

    public static class MotorPIDValuesExample {
        public static double kP = 0.003;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;
        public static double MAX_VELOCITY = 0;
        public static double ACCELERATION = 0;
        public static double DECELERATION = 0;
        public static double TOLERANCE = 5;
    }

    public static class MotorPIDGainsExample {
        public static PIDGains EXAMPLE_MOTOR_PID = new PIDGains(MotorPIDValuesExample.kP, MotorPIDValuesExample.kI, MotorPIDValuesExample.kD);
        public static ProfileConstraints Example_Motor_PC = new ProfileConstraints(MotorPIDValuesExample.MAX_VELOCITY, MotorPIDValuesExample.ACCELERATION, MotorPIDValuesExample.DECELERATION);
    }

    @Config
    public static class ElevatorJointPIDValues {
        public static double kP = 0.009;
        public static double kI = 0.002;
        public static double kD = 0.0001;
        public static double kF = 0.0009;
        public static double MAX_VELOCITY = 34;
        public static double ACCELERATION = 170;
        public static double DECELERATION = 120;
        public static double TOLERANCE = 3;
    }

    @Config
    public static class ElevatorAngles {

        public static double SCORE = 400;
        public static double INTAKE = 1;
        public static double PARALLEL = 100;

    }


    @Config
    public static class ElevatorPIDValues {
        public static double kP = 0.0035;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;
        public static double MAX_VELOCITY = 230;
        public static double ACCELERATION = 600;
        public static double DECELERATION = 320;
        public static double TOLERANCE = 3;
    }

    @Config
    public static class ElevatorHeight {

        public static double ZERO = 1;
        public static double MIDDLE = 900;
        public static double HIGH = 1800;

    }

    @Config
    public static class ServoAngle {
        public static double INTAKE = 0.3;
        public static double SCORE_ONE = 1;
        public static double SCORE_TWO = 0.3;
        public static double HOLD = 0.8;

    }

    @Config
    public static class IntakePositions {
        public static double OPEN = 0.75;
        public static double CLOSED = 1;

    }

    @Config
    public static class ClimberPosition {
        public static double OPEN = 0.7;
        public static double CLOSED = 1;

    }

    @Config
    public static class CameraSettings {
        public static int GAIN = 0;
        public static int EXPOSURE = 5  ;
        public static final double FX = 505.589;
        public static final double FY = 505.589;
        public static final double CX = 335.15;
        public static final double CY = 244.737;
        public static final int WIDTH = 640;
        public static final int HEIGHT = 480;

    }

}
