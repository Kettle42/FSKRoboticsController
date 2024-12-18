package org.firstinspires.ftc.teamcode.DeepDive;

import android.annotation.TargetApi;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class SampleIdentifier
{
    private final RevColorSensorV3 colorSensor;

    public SampleIdentifier(RevColorSensorV3 colorSensor)
    {
        this.colorSensor = colorSensor;
    }

    @TargetApi(26)
    public Color getColor()
    {
        return Color.valueOf(colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }
}
