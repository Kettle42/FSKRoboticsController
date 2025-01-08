package org.firstinspires.ftc.teamcode.DeepDive;

public class HandValues
{
    public enum ClawMode
    {
        Clippy(0.00, +0.112),
        Grabby(0.77, -0.330);

        public final double position;
        public final double openOffset;

        ClawMode(double pos, double openOffset)
        {
            this.position = pos;
            this.openOffset = openOffset;
        }

        public ClawMode toggle()
        {
            if (this == Clippy) return Grabby;
            return Clippy;
        }
    }

    public enum WristMode
    {
        Folded(0.00),
        Clippy(0.73),
        Grabby(1.00);

        public final double position;

        WristMode(double pos)
        {
            this.position = pos;
        }

        public WristMode toggle()
        {
            if (this == Clippy) return Grabby;
            return Clippy;
        }
    }
}
