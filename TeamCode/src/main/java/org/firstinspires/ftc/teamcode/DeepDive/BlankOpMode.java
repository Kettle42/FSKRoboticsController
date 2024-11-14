package org.firstinspires.ftc.teamcode.DeepDive;

import androidx.appcompat.app.ActionBar;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Blank Test")
@Disabled
public class BlankOpMode extends LinearOpMode
{
    public GoBildaPinpointDriver brains;

    public void runOpMode()
    {
        brains = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        brains.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        brains.setOffsets(-141.0, -141.0);
        brains.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
//        brains.setEncoderDirections();
        brains.resetPosAndIMU();
//        brains.re

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                brains.bulkUpdate();
                telemetry.addData("X Encoder", brains.getEncoderX());
                telemetry.addData("Y Encoder", brains.getEncoderY());
                telemetry.addLine("Position: " + brains.getPosX() + ", " + brains.getPosY());
                telemetry.addData("Heading: ", brains.getHeading());
                telemetry.update();
            }

        }
    }
}
