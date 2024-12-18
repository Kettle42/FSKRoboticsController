package org.firstinspires.ftc.teamcode.KettleLibrary;

import com.qualcomm.robotcore.hardware.Servo;

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
}
