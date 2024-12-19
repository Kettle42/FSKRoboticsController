package org.firstinspires.ftc.teamcode.KettleLibrary;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DeepDive.SampleIdentifier.Color;

import java.security.Provider;

public enum ColorBrickColor
{
    Off   (0.000),
    Red   (0.277),
    Orange(0.333),
    Yellow(0.388),
    Sage  (0.444),
    Green (0.500),
    Azure (0.555),
    Blue  (0.611),
    Indigo(0.666),
    Violet(0.722),
    White (1.000);

    public final double pos;

    ColorBrickColor(double p)
    {
        this.pos = p;
    }

    public void setColorBrick(Servo light)
    {
        light.setPosition(this.pos);
    }

    public Color toColor()
    {
        if (this.pos < 0.277) return new Color(0, 0, 0);
        if (this.pos > 0.722) return new Color(255, 255, 255);

        double hue = (this.pos - 0.277) * (270.0 / 0.445);
        int X = 255 * (int)((1 - Math.abs(((hue / 60) % 2) - 1)));

        if (hue < 60)
        {
            return new Color(255, X, 0);
        }
        else if (hue < 120)
        {
            return new Color(X, 255, 0);
        }
        else if (hue < 180)
        {
            return new Color(0, 255, X);
        }
        else if (hue < 240)
        {
            return new Color(0, X, 255);
        }
        else if (hue < 300)
        {
            return new Color(X, 0, 255);
        }
        else if (hue < 360)
        {
            return new Color(255, 0, X);
        }

        return new Color(0, 0, 0);
    }
}
