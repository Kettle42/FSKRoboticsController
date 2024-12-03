package org.firstinspires.ftc.teamcode.KettleLibrary;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

public class XyhVector
{
    public float x;
    public float y;
    public float h;

    public XyhVector(float x, float y, float h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public XyhVector(XyhVector other)
    {
        this.x = other.x;
        this.y = other.y;
        this.h = other.h;
    }

    public XyhVector copy()
    {
        return new XyhVector(this);
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString()
    {
        return String.format("XyhVector( %f, %f, %f )", this.x, this.y, this.h);
    }


}
