package org.firstinspires.ftc.teamcode.DeepDive;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Vision {
    private WebcamName webcam;

    private AprilTagProcessor aprilProcess;
    private VisionPortal visPort;
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    public Vision(WebcamName webcam)
    {
        this.webcam = webcam;
        initProcessor();
    }

    private void initProcessor()
    {
        AprilTagProcessor.Builder aprilBuilder = new AprilTagProcessor.Builder();
        aprilProcess = aprilBuilder.build();

        VisionPortal.Builder visionBuilder = new VisionPortal.Builder();
        visionBuilder.setCamera(webcam);
        visionBuilder.setCameraResolution(new Size(1280, 720));
        visionBuilder.addProcessor(aprilProcess);
        visionBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        visPort = visionBuilder.build();
    }

    public AprilTagDetection[] detect()
    {
        detections = aprilProcess.getDetections();
        return detections.toArray(new AprilTagDetection[detections.size()]);
    }

    public AprilTagDetection tryGetTag(int id)
    {
        detect();
        for (AprilTagDetection detection : detections)
        {
            if (detection.id == id)
                return detection;
        }
        return null;
    }
}
