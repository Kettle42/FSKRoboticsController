package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KettleLibrary.ColorBrickColor;

@TeleOp(name = "Color Sensor Test")
public class ColorSensorToLightTest extends LinearOpMode
{
    Servo light;
    RevColorSensorV3 colorSensor;

    int[] yellow = new int[]{127,127,0};
    int[] red    = new int[]{127,0,  0};
    int[] blue   = new int[]{0,  0,127};

    public void runOpMode()
    {
        light = hardwareMap.get(Servo.class, "light");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive()) {
                int redDetected   = colorSensor.red();
                int greenDetected = colorSensor.green();
                int blueDetected  = colorSensor.blue();

                double yellowMag = Math.sqrt(Math.pow(yellow[0] - redDetected,2) +
                                             Math.pow(yellow[1] - greenDetected,2) +
                                             Math.pow(yellow[2] - blueDetected,2)  );

                double redMag = Math.sqrt(Math.pow(red[0] - redDetected,2) +
                                          Math.pow(red[1] - greenDetected,2) +
                                          Math.pow(red[2] - blueDetected,2)  );

                double blueMag = Math.sqrt(Math.pow(blue[0] - redDetected,2) +
                                           Math.pow(blue[1] - greenDetected,2) +
                                           Math.pow(blue[2] - blueDetected,2)  );

                double minMag = Math.min(Math.min(yellowMag, redMag), blueMag);
                double distance = colorSensor.getDistance(DistanceUnit.MM);

                telemetry.addLine(distance + " millimeters");
                if (distance <= 60.0) {
                    if (minMag == blueMag) {
                        telemetry.addLine("BLUE");
                        light.setPosition(0.63);
//                        light.setPosition(ColorBrickColor.Blue.pos);
                        ColorBrickColor.Blue.setColorBrick(light);
                    } else if (minMag == redMag) {
                        telemetry.addLine("RED");
                        light.setPosition(0.28);
                    } else {
                        telemetry.addLine("YELLOW");
                        light.setPosition(0.34);
                    }
                } else {
                    telemetry.addLine("Cannot provide an accurate read");
                    light.setPosition(0.0);
                }

                telemetry.update();
            }
        }
    }
}
