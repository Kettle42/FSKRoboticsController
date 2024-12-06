# The Use of the KettleLibrary Package

This package is for cross-season utilites. Such as our PIDController class, Vision class, or XyhVector. 
Use this for any backend systems that are generalized and can be used in any year.  

***When you update or add a class, add documentation here. This is not a request.***

---

# Code Systems

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
To make a Vision object, you will need a `WebcamName` (which is just a webcam on the Robot, gotten with 
`hardwareMap.get(WebcamName.class, "WEBCAM_NAME_HERE")`), the lens intrinsics
(see [here](https://www.youtube.com/watch?v=bTcCY3DZM0k "Tutorial on finding lens intrinsics") for a tutorial), 
the camera offset (x, y, z position, and yaw, pitch, roll rotation), and the resolution.

#### Setting up and using a Vision object

With the lens intrinsics, use the Vision.LensIntrinsics class. ***If you have to find the intrinsics 
for a camera, add the intrinsics as a static variable in the class for future use.***  

For the camera offset, measure the forward offset in millimeters for `x`, the sideways offset in millimeters
for `y`, and there should be no need for the `z` offset (unless you want to be very particular). Next, 
you will need the orientation, measured in degrees. The `yaw` is how much the camera is rotated parallel 
to the ground (or it could be thought of as turning left or right). The `pitch` is how much the camera is 
rotated perpendicular to the front of the Robot (like looking up or down). And `roll` is how much the camera is 
rotated parallel to the front of the Robot (like rotating to see something that is sideways). 

For the resolution, make a new `Size` object with the resolution of the camera (e.g. `new Size(640, 480)`).

For getting AprilTag data, use `vision.detect();`. This will return an array of the seen AprilTags, so 
to get a specific index of a tag, use `vision.detect()[index];`. If you need to get the same list of 
without gathering camera input, use `vision.getLastDetections();`. This will return that same array of 
tags. 

To try and get a specific tag by an id, use `vision.tryGetTag(id);` This **will** rescan for inputs.
This returns the specific AprilTagDetection object if it can be found, otherwise it returns `null`.

--- 

## Other Code Systems

### The PIDController class

The purpose of this class is to allow us to set a target position for a motor or Robot Position and 
approach that position without overshooting. Unless you are very interested, you do not need to know 
exactly how this class works. See 
[here](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller "PID Controller Explanation")
if you really want to know.

#### Setting up a PIDController

To make a PIDController, all you need to do is call its constructor.
`PIDController pid = new PIDController();`

Next step is to set coefficients. Most should start with something close to 
`pid.setCoefficients(0.05. 0.0, 0.0);`

Now the only thing left is application of the PID and its output.

#### For basic use

For basic use, it is good enough to use only one value. The way we have found works best to do this 
is to use the cube root of the error. Here is an example.

```
int currentState = motor.getCurrentPosition();
int targetPosition = 1000;

int error = targetPosition - currentState;
double cbrtErr = Math.cbrt(error);

motor.setPower(pid.update(cbrtErr));
```

This should let your motor approach encoder position 1000 without going over.

#### For Adavanced use

[//]: # (TODO: Add tutorial for tuning PID Coefficients, what Ki and Kd do, etc.)

---

# Code Structuring

---

## TeleOp

For TeleOp, it is a good idea to have a basic flow for your program. It should start with defining the 
variables for Robot actuators/sensors, and any other instance variables needed in different methods. 
It should then contain your `runOpMode()` method, where you first get the actual objects for the actuators/sensors
using `hardwareMap.get()`, and . It then should call `waitForStart();`, check if `opModeIsActive()` 
and enter a loop that runs while `opModeIsActive()`. 

Here is a good sample so far...
```java
public class 
```