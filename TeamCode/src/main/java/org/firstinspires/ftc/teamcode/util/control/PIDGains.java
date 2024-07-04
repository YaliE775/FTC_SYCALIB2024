package org.firstinspires.ftc.teamcode.util.control;

public class PIDGains {
    private double[] gains;
    public PIDGains(double kP, double kI, double kD) {
        gains = new double[] {kP, kI, kD};
    }
    public double getProportional() {
        return gains[0];
    }

    public double getIntegral() {
        return gains[1];
    }

    public double getDerivative() {
        return gains[2];
    }

}
