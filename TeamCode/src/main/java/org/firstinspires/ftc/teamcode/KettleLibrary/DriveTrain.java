package org.firstinspires.ftc.teamcode.KettleLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain
{
    public static enum Wheel
    {
        FrontLeft ,
        FrontRight,
        BackLeft  ,
        BackRight,
    }

    public final DcMotor frontleft;
    public final DcMotor frontright;
    public final DcMotor backleft;
    public final DcMotor backright;
    private boolean initialized = false;

    public DriveTrain(HardwareMap hardwareMap)
    {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
    }

    public void finishInit(Wheel reverse1, Wheel reverse2)
    {
        initialized = true;

    }

    public void drive(Gamepad driver, double power)
    {
        double lx = driver.left_stick_x;
        double ly = driver.left_stick_y;
        double rx = driver.right_stick_x;

        frontleft.setPower((ly + lx + rx) * power);
        frontright.setPower((ly - lx - rx) * power);
        backleft.setPower((ly - lx + rx) * power);
        backright.setPower((ly + lx - rx) * power);
    }
}