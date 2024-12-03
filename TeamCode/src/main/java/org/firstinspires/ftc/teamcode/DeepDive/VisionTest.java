package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.KettleLibrary.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import java.lang.Math;

@TeleOp(name = "Vision Test")
@Disabled
public class VisionTest extends LinearOpMode
{
    Vision vision;

    @Override
    public void runOpMode()
    {
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        vision = new Vision(webcam);

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                AprilTagDetection[] detections = vision.detect();
                for (AprilTagDetection detection : detections)
                {
                    if (detection == null)
                        continue;

                    AprilTagPoseFtc pose = detection.ftcPose;
                    if (pose == null) continue;

                    telemetry.addLine(detection.id + "-----------------------------");
                    telemetry.addData("range", pose.range);
                    telemetry.addData("bearing", pose.bearing);
                    telemetry.addData("yaw", pose.yaw);

                    double theta = pose.yaw; // degrees
                    theta *= (Math.PI / 180); // degrees -> radians
                    double xOff = pose.range * Math.cos(theta); // radians
                    double yOff = pose.range * Math.sin(theta);
                    telemetry.addLine("Offset Pos: " + xOff + ", " + yOff);
                }
                telemetry.addLine("------------------------------------");
                telemetry.update();
            }
        }
    }
}
