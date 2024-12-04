# The Use of the KettleLibrary Package

This package is for cross-season utilites. Such as our PIDController class, Vision class, or XyhVector. 
Use this for any backend systems that are generalized and can be used in any year.  

***When you update or add a class, add documentation here. This is not a request.***

---

## Code Systems

---

## Odometry Systems

### The GoBildaPinpointDriver

This class is used to communicate with the GoBilda dead-wheel odometry (either the swingarms or the 4-bars)
There is no need to import this or create an object for this if you do not use the GoBildaPinpoint IMU.

#### Setting Up and Using the GoBildaPinpointDriver

To use this, make sure that the Robot configuration has the IMU in an I2C bus port (preferrably 2 and 3)
and has a name. In your class, before the `runOpMode()` method, make an instance variable for the IMU 
(`GoBildaPinpointDriver imu;`) 
Then, in the `runOpMode()` method in your Robot program, assign the IMU to the instance variable,  
`imu = hardwareMap.get(GoBildaPinpointDriver.class, "YOUR_CONFIG_NAME_HERE");`.  

Now, you need to set the offsets, by measuring how far the dead-wheels are from the center of the Robot. 
Keep in mind the orientation of the IMU because that will matter.  

Now, you need to set the encoder resolution. This should be provided in an enum, 
(`GoBildaPinpointDriver.GoBildaOdometryPods`, and then either `goBILDA_SWINGARM_POD` or `goBILDA_4_BAR_POD`).
Otherwise, you may need to go to the GoBilda website and gather the encoder resolution from there. 

Now, you need to figure out the encoder directions. 
This could be difficult, so it is recommended that you do this experimentally. 
Run your OpMode and move the robot forward, and use the telemetry to see how the encoder changed. 
If it is negative, you can either reverse the proper direction, or, you can just leave negative as forward (or vice versa).

For getting the IMU initialized with the OpMode, run `imu.resetPosAndIMU();`. After `waitForStart();`, in your while loop, 
you should be able to get the tracked position of the Robot using `imu.getPosition();`.  

### The SparkFun Optical Tracking Odometry Sensor (Laser Odometry)

This class is for communicating with the laser odometry. 
It is not required to configure this or instantiate it if you are not using this device.

#### Setting up and using the SparkFunOTOS

To start, ensure that the sensor is configured. It should be under an I2C bus port (preferrably 2 or 3) 
as a SparkFunOTOS. In your class, before the `runOpMode()` method, create an instance variable for the sensor
(`SparkFunOTOS otos;`).
In the `runOpMode()` method, assign the sensor to the variable, 
`otos = hardwareMap.get(SparkFunOTOS.class, "YOUR_CONFIG_NAME_HERE");`.

Now, you need to configure the sensor. 

First, you need the offset from the center of the robot. Measure this, then construct an offset
variable as a `SparkFunOTOS.Pose2D` with units, an `x` and a `y`, and then `otos.setOffset(offset);`

Now, you need to find the linear and angular scalars. For this, you move the robot an amount. Keep this 
amount in mind, because it will be used. For example, move the robot 5 meters. 
Then, look at how far the robot think it moved. To set the scalar, use `otos.setLinearScalar(ACTUAL_DISTANCE / MEASURED_DISTANCE);` 
or `otos.setAngularScalar(ACTUAL_ROTATION / MEASURED_ROTATION);`

Now you need to run these commands:
```
otos.calibrateIMU();
otos.resetTracking();
```

The sensor should be all calibrated and ready to go now.

To get a position from the sensor, use `otos.getPosition();`. This will return a `SparkFunOTOS.Pose2D`
with an `x`, `y`, and heading.

--- 

## Vision

### The Vision class

The purpose of this class is to abstract all of the systems that are required to get AprilTag inputs. 
To make a Vision object, you will need a `WebcamName` (which is just a webcam on the Robot), the lens intrinsics
(see [here](https://www.youtube.com/watch?v=bTcCY3DZM0k "Tutorial on finding lens intrinsics") for a tutorial), 
the camera offset (x, y, z position, and yaw, pitch, roll rotation), and the resolution.