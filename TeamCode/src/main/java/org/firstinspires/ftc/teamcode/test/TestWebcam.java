package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.herev.Webcam;
import org.firstinspires.ftc.teamcode.util.opmodes.TeleopBase;

@TeleOp
public class TestWebcam extends TeleopBase {
    @Override
    public void configureBindings() {
        Webcam cam = new Webcam(hardwareMap, telemetry);
        cam.setCameraControls();
    }
}
