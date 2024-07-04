package org.firstinspires.ftc.teamcode;

public class Globals {
    private static volatile Globals instance;

    public enum ClawAngleMode{
        SCORE_ONE,
        SCORE_TWO,
        INTAKE,
        HOLD;

    }

    public enum ClawIntakeMode{
        CLOSED,
        OPENED;
    }

    public enum ElevatorJointMode{
        SCORE,
        INTAKE;
    }

    public enum ElevatorMode{
        ZERO,
        MIDDLE,
        HIGH;
    }

    public enum RobotMode {
        AUTO_END,
        AUTO_START,
        GO_INTAKE,
        INTAKE,
        GO_SCORE,
        SCORE,
        CLIMB;
    }

    public enum AllianceColor{
        BLUE,
        RED;
    }

    public static Globals getInstance() {
        if(instance == null) {
            synchronized (Globals.class) {
                if(instance == null) {
                    instance = new Globals();
                }

            }
        }
        return instance;
    }


    public ClawAngleMode clawAngleMode = ClawAngleMode.HOLD;
    public ClawIntakeMode rightIntakeMode = ClawIntakeMode.CLOSED;
    public ClawIntakeMode leftIntakeMode = ClawIntakeMode.CLOSED;
    public ElevatorJointMode elevatorJointMode = ElevatorJointMode.INTAKE;
    public ElevatorMode elevatorMode = ElevatorMode.ZERO;
    public RobotMode robotMode = RobotMode.AUTO_END;


    void changeClawAngleState(ClawAngleMode clawAngleMode) {
        this.clawAngleMode = clawAngleMode;
    }

    void changeLeftClawIntakeState(ClawIntakeMode clawIntakeMode) {
        this.leftIntakeMode = clawIntakeMode;
    }

    void changeRightClawIntakeState(ClawIntakeMode clawIntakeMode) {
        this.rightIntakeMode = clawIntakeMode;
    }

    void changeElevatorJointState(ElevatorJointMode elevatorJointMode) {
        this.elevatorJointMode = elevatorJointMode;
    }
    void changeElevatorState(ElevatorMode elevatorMode) {
        this.elevatorMode = elevatorMode;
    }

    void changeRobotState(RobotMode robotMode) {
        this.robotMode = robotMode;
    }



}
