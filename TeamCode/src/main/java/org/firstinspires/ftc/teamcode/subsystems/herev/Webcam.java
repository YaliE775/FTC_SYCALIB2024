package org.firstinspires.ftc.teamcode.subsystems.herev;

import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class Webcam extends SubsystemBase {
    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;
    Telemetry telemetry;

    public Webcam(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(Constants.CameraSettings.WIDTH, Constants.CameraSettings.HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .build();

        this.telemetry = telemetry;
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void setCameraControls() {
        boolean isCameraReady = false;

        while(!isCameraReady) {
            if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                isCameraReady = true;
            }
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(Constants.CameraSettings.GAIN);
        exposureControl.setExposure(Constants.CameraSettings.EXPOSURE, TimeUnit.MILLISECONDS);

    }

    @Override
    public void periodic() {
        if(aprilTagProcessor.getDetections() != null && !aprilTagProcessor.getDetections().isEmpty()) {
            telemetry.addData("X:", aprilTagProcessor.getDetections().get(0).ftcPose.x);
            telemetry.addData("Y:", aprilTagProcessor.getDetections().get(0).ftcPose.y);

        }
    }
}
