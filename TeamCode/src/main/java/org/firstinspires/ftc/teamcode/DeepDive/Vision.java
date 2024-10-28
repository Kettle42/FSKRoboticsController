package org.firstinspires.ftc.teamcode.DeepDive;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
        Position cameraPosition = new Position(DistanceUnit.INCH, 0.0, 7.36417323, 4.4375, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0);
        AprilTagProcessor.Builder aprilBuilder = new AprilTagProcessor.Builder();
        /*
        These intrinsics are specific to the wide angle lens
        if you are not using that lens, comment the next line out
        or follow a tutorial to get that specific cameras intrinsics
        */
        aprilBuilder.setLensIntrinsics(539.415, 539.415, 625.669, 385.219);
        aprilBuilder.setCameraPose(cameraPosition, cameraOrientation);
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

    public AprilTagDetection[] getLastDetections()
    {
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
