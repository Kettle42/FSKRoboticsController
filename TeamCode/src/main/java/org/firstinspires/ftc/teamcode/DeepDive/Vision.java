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
        // Cameras position on the big testing robot
        Position testingCameraPosition = new Position(DistanceUnit.INCH, 0.0, 7.36417323, 4.4375, 0);
        YawPitchRollAngles testingCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0);

        Position competitionCameraPosition = new Position(DistanceUnit.INCH, 0.0, 6.02362, 2.32283, 0);
        YawPitchRollAngles competitionCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, -90.0, 0);

        AprilTagProcessor.Builder aprilBuilder = new AprilTagProcessor.Builder()
                /*.setLensIntrinsics(500.0, 500.0, 640.0, 360.0) */ // Testing robot lens intrinsics
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502) // Logitech C270 webcam
                .setDrawAxes(true)
                .setCameraPose(competitionCameraPosition, competitionCameraOrientation);

        aprilProcess = aprilBuilder.build();
        aprilProcess.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE);


        VisionPortal.Builder visionBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                /*.setCameraResolution(new Size(1280, 720))*/ // Testing Robot resolution
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilProcess)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG);

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
