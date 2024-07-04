package org.firstinspires.ftc.teamcode.util.control;

import com.arcrobotics.ftclib.controller.PIDController;

public class PIDControllerS extends PIDController {

    public PIDControllerS(PIDGains gains ) {
        super(gains.getProportional(), gains.getIntegral(), gains.getDerivative());
    }

}
