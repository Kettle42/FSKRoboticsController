package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.KettleLibrary.ColorBrickColor;

public class SampleIdentifier
{
    public static class Color
    {
        public final int r;
        public final int g;
        public final int b;

        private static int validate(int n)
        {
            return Math.max(Math.min(n, 255), 0);
        }

        public Color(int r, int g, int b)
        {
            this.r = (byte)validate(r);
            this.g = (byte)validate(g);
            this.b = (byte)validate(b);
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

        public double distance(Color other)
        {
            return (this.subtract(other)).magnitude();
        }

        public String toString()
        {
            return String.format("Color( %d, %d, %d )", r, g, b);
        }
    }

    public static enum SampleColor
    {
        Red,
        Blue,
        Yellow;

        public ColorBrickColor toColorBrickColor()
        {
            if (this == Red) return ColorBrickColor.Red;
            if (this == Blue) return ColorBrickColor.Blue;
            return ColorBrickColor.Yellow;
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

    public SampleColor getSampleColor()
    {
        Color color = getColor();
        Color[] colors = new Color[]
        {
            new Color(255, 0, 0),
            new Color(0, 0, 255),
            new Color(255, 255, 0)
        };
        Color currentLow = colors[0];
        double lowestdist = colors[0].distance(color);

        for (Color testColor : colors)
        {
            if (testColor.distance(color) < lowestdist)
            {
                currentLow = testColor;
                lowestdist = testColor.distance(color);
            }
        }

        if (currentLow == colors[0]) return SampleColor.Red;
        else if (currentLow == colors[1]) return SampleColor.Blue;
        return SampleColor.Yellow;
    }

    public void displaySampleColor(Servo light)
    {
        getSampleColor().toColorBrickColor().setColorBrick(light);
    }
}