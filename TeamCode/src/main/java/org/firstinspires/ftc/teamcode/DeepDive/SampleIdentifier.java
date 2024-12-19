package org.firstinspires.ftc.teamcode.DeepDive;

import android.annotation.TargetApi;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class SampleIdentifier
{
    public static class Color
    {
        public final byte r;
        public final byte g;
        public final byte b;

        public Color(int r, int g, int b)
        {
            this((byte)r, (byte)g, (byte)b);
        }

        public Color(byte r, byte g, byte b)
        {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public Color subtract(Color other)
        {
            return new Color(this.r - other.r, this.g - other.g, this.b - other.b);
        }

        public Color add(Color other)
        {
            return new Color(this.r + other.r, this.g + other.g, this.b + other.b);
        }

        public double magnitude()
        {
            return Math.sqrt
            (
                Math.pow(this.r, 2.0) +
                Math.pow(this.g, 2.0) +
                Math.pow(this.b, 2.0)
            );
        }
    }

    private final RevColorSensorV3 colorSensor;

    public SampleIdentifier(RevColorSensorV3 colorSensor)
    {
        this.colorSensor = colorSensor;
    }

    public Color getColor()
    {
        return new Color(colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }
}
