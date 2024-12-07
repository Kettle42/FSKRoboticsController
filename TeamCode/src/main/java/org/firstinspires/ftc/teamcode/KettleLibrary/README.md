# The Use of the KettleLibrary Package

This package is for cross-season utilites. Such as our PIDController class. Use this for any backend
systems that are generalized and can be used in any year.  

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
`pid.setCoefficients(0.5. 0.0, 0.0);`

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
Something to note, however, is that you still have to tune the proportional coefficient
of the PID so that it doesn't go too fast or too slow, this is best accomplished by
starting with a value like 0.5, if thats too fast then halve it to 0.25, if its too
slow, multiply by 1.5 to 0.75. Repeat this process until the desired effect is reached.

#### For Adavanced use

For advanced usage, we will first need to discuss what the four coefficients do.

Kp is the proportional coefficient, it allows for the system to adjust based on
the amount of error from the target point. This helps the the system adjust to try and
reach a set point. For example, when the robot is trying to turn so that is is facing
an april tag, it might not know how much to turn or how that affects the amount of
power given to the motors. The Kp coefficient allows for the robot to understand its
error and start correcting and moving towards a specific point.

Kd is the derivative coefficient, it allows for the system to adjust based on the
rate of change. This helps to prevent overshoot and improve stability by preventing
rapid changes of the system. For example, when the robot is trying to turn so that 
it is facing an april tag, it might turn too much and then have to turn the other 
direction to compensate. The Kd coefficient would drastically reduce that oscillation.

Ki is the integral coefficient, it allows the system to adjust based on the accumulated
error over the run time of the system. This allows the system to correct any small 
remaining error that might exist when the system gets closer to the target. This
allows for a more precise and accurate system. For example, when the robot is trying
to turn so that it is facing an april tag, it might be stuck turned 10 degrees away
from the april tag. The Ki coefficient would help the robot be able to turn that
extra 10 degrees and be more accurate.

Kf is the feedforward coefficient, it allows the system to adjust based on any estimated
disturbances. This allows for the system to maintain a steady path or go back to the 
target point more reliably if moved away from it. For example, when the robot is trying
to turn so that it is facing an april tag, it could bump into another robot and suddenly
get stuck a lot further from the target point. The Kf coefficient would help the robot
calculate for disturbances and be able to adjust afterwards.

Now that we have a basic understanding of what each coefficient does, how do we tune 
the PID for a desired effect.

Start by setting everything to 0 except for proportional coefficient, which should be 0.5. 

[//]: # (TODO: section on how to tune a PID)

---
# Code Structuring

---

## TeleOp

For TeleOp, it is a good idea to have a basic flow for your program. Your class should start with defining the 
variables for Robot actuators/sensors, and any other instance variables needed in different methods. 
The class should then contain your `runOpMode()` method, where you first get the actual objects for the actuators/sensors
using `hardwareMap.get()`, and . The method then should call `waitForStart();`, check if `opModeIsActive()` 
and enter a loop that runs while `opModeIsActive()`. 

Here is a good sample so far...

```java
/*
    SeasonFolder/RobotProgram.java
    
    12-6-2024 by Matthew Perry
 */

package org.firstinspires.ftc.teamcode.SeasonFolder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// ^ these imports will be expanded upon

@TeleOp(name = "Robot Program") // this marks the program as a TeleOp OpMode
// ^ you can change the name that is displayed on the Driver Station here
public class RobotProgram extends LinearOpMode 
{
    public DcMotor frontleft;
    public DcMotor frontright;
    public DcMotor backleft;
    public DcMotor backright;
    // make the wheels accessible from all Robot methods

    public int someGlobalVar = 24;
    // ^ this variable will be accessible from any method in the TeleOp routine 

    @Override
    public void runOpMode() 
    {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        // ^ get the wheel objects from the Robot configuration
        // if you named the wheels something different in the Robot configuration, 
        // make sure that you change these Strings accordingly

        // more setup code will go here later

        double someLocalVar = 20.5;
        // ^ this variable will only be accessible in this method
        // trying to access it in another method will result in a crash

        // ^ all of the above code runs when INIT is pressed
        waitForStart(); // waits for the start button to be pressed
        if (opModeIsActive()) 
        {
            while (opModeIsActive()) 
            {
                // TeleOp code will go here
            }
        }
    }
}
```

The next step is to get the Robot driving. assuming you are using mecanum wheels, you will need to 
know how they work. The first thing you need to do is to reverse the proper wheels so that the Robot 
will go forward when all wheels are set to positive power. An easy way to do this is to tell the Robot
under `// TeleOp code will go here` to set all wheel power to 0.5.

```
// TeleOp code will go here
frontleft.setPower(0.5);
frontright.setPower(0.5);
backleft.setPower(0.5);
backright.setPower(0.5);
```

When you run the program, you can lift the Robot off the ground to see which of the wheels are moving 
backwards, if any. These motors should be reversed in the program. To do this, go to where it says 
`// more setup code will go here later` and insert `wheel.setDirection(DcMotorSimple.Direction.REVERSE)`
for each wheel that needs reversing (You may need to add `import com.qualcomm.robotcore.hardware.DcMotorSimple;` 
for this). Now, when you run that program, all the wheels should hopefully be going the same way. If
you now place the Robot on the ground, it should move forwards. If it did not, remove the 
`wheel.setDirection()` lines and try again.

If your Robot went forward, congratulations! Now onto the hard part. Mecanum wheels are tricky when 
it comes to getting them to operate properly. 