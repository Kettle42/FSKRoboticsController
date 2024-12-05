package org.firstinspires.ftc.teamcode.KettleLibrary;

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
    private final WebcamName webcam;

    private AprilTagProcessor aprilProcess;
    private VisionPortal visPort;
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    public static class LensIntrinsics
    {
        public static LensIntrinsics LogitechC270 = new LensIntrinsics(822.317, 822.317, 319.495, 242.502);
        // add new lens intrinsics here ^^

        public double fx;
        public double fy;
        public double cx;
        public double cy;

        public LensIntrinsics(double fx, double fy, double cx, double cy)
        {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
        }
    }

    public static class CameraOffset
    {
        public Position position;
        public YawPitchRollAngles orientation;

        public CameraOffset(double xMM, double yMM, double zMM, double yawDeg, double pitchDeg, double rollDeg)
        {
            position = new Position(DistanceUnit.MM, xMM, yMM, zMM, 0);
            orientation = new YawPitchRollAngles(AngleUnit.DEGREES, yawDeg, pitchDeg, rollDeg, 0);
        }
    }

    public Vision(WebcamName webcam, LensIntrinsics lens, CameraOffset offset, Size resolution)
    {
        this.webcam = webcam;
        initProcessor(lens, offset, resolution);
    }

    private void initProcessor(LensIntrinsics lens, CameraOffset offset, Size resolution)
    {
        AprilTagProcessor.Builder aprilBuilder = new AprilTagProcessor.Builder()
                /*.setLensIntrinsics(500.0, 500.0, 640.0, 360.0) */ // Testing robot lens intrinsics
                .setLensIntrinsics(lens.fx, lens.fy, lens.cx, lens.cy) // Logitech C270 webcam
                .setDrawAxes(true)
                .setCameraPose(offset.position, offset.orientation);

        aprilProcess = aprilBuilder.build();
        aprilProcess.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE);

        VisionPortal.Builder visionBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                /*.setCameraResolution(new Size(1280, 720))*/ // Testing Robot resolution
                .setCameraResolution(resolution)
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
