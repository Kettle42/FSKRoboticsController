package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.KettleLibrary.ColorBrickColor;

@TeleOp
public class ColorSensorToLightTestTwo extends LinearOpMode
{
    private Servo light;
    private RevColorSensorV3 colorSensor;

    private SampleIdentifier sampleIdentifier;

    @Override
    public void runOpMode()
    {
        light = hardwareMap.get(Servo.class, "light");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        sampleIdentifier = new SampleIdentifier(colorSensor);

        waitForStart();

        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                SampleIdentifier.SampleColor identified = sampleIdentifier.getSampleColor();

//                for (ColorBrickColor color : ColorBrickColor.values())
//                {
//                    telemetry.addData(color.toString(), color.toColor());
//                }

                telemetry.addData("seen", identified);
                telemetry.addData("actual color", sampleIdentifier.getColor().toString());

                identified.toColorBrickColor().setColorBrick(light);

                telemetry.update();
            }
        }
    }
}
