package org.firstinspires.ftc.teamcode.KettleLibrary;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainSparkFun extends DriveTrain
{
    public final SparkFunOTOS otos;

    public DriveTrainSparkFun(HardwareMap hardwareMap)
    {
        super(hardwareMap);
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
    }

    public SparkFunOTOS.Pose2D getPosition()
    {
        return otos.getPosition();
    }
}
