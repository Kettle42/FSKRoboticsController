package org.firstinspires.ftc.teamcode.KettleLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain
{
    public final DcMotor frontleft;
    public final DcMotor frontright;
    public final DcMotor backleft;
    public final DcMotor backright;

    public DriveTrain(HardwareMap hardwareMap)
    {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
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
