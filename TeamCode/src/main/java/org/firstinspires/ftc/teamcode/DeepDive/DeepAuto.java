package org.firstinspires.ftc.teamcode.DeepDive;

import java.util.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KettleLibrary.PIDController;
import org.firstinspires.ftc.teamcode.KettleLibrary.Vision;

// import org.firstinspires.ftc.teamcode.KettleLibrary.XyhVector;

@Autonomous(name = "Deep Dive v1.0.0")
public class DeepAuto extends LinearOpMode
{
    public enum AutoState
    {
        Start,
        RaiseArm,
        ApproachBar,
        ExtendArm,
        ReleaseArm,
        Park,
        TouchBar,
        End;
    }

    public enum TouchBarSubStage
    {
        MoveBackLeft,
        MoveForward,
        MoveRight,
        MoveArm,
    }
    // matthew was here :)
    public static class PoseMath
    {
        public static SparkFunOTOS.Pose2D add(SparkFunOTOS.Pose2D a, SparkFunOTOS.Pose2D b)
        {
            return new SparkFunOTOS.Pose2D(a.x + b.x, a.y + b.y, a.h + b.h);
        }

        public static SparkFunOTOS.Pose2D subtract(SparkFunOTOS.Pose2D a, SparkFunOTOS.Pose2D b)
        {
            return new SparkFunOTOS.Pose2D(a.x - b.x, a.y - b.y, a.h - b.h);
        }

        public static double distance(SparkFunOTOS.Pose2D a, SparkFunOTOS.Pose2D b)
        {
            return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
        }
    }

    private DcMotor frontright;
    private DcMotor frontleft;
    private DcMotor backright;
    private DcMotor backleft;

    private DcMotor tricep;
    private DcMotor shoulder;

    private Servo wrist;
    private Servo hand;

    SparkFunOTOS odo;

    AutoState state;
    TouchBarSubStage touchBarState;


    double oldTime = 0;
    double xOffset = 26; //  152 is Testing Robot offset
    double yOffset = 92; // -140 is Testing Robot offset

    double cameraOffset = 187.05;

    ElapsedTime xPosTimer;
    ElapsedTime yPosTimer;
    ElapsedTime hPosTimer;

    PIDController xPosPID;
    PIDController yPosPID;
    PIDController hPosPID;

    PIDController shoulderPID;
    PIDController tricepPID;

    Vision vision;

    boolean bar;

    SparkFunOTOS.Pose2D targetPosition;

    /*static OurPose2D[] tagXYH = new OurPose2D[] { new OurPose2D(DistanceUnit.INCH, -72.0,  48.0, AngleUnit.DEGREES, 180.0),
            new OurPose2D(DistanceUnit.INCH,   0.0,  72.0, AngleUnit.DEGREES,  90.0),
            new OurPose2D(DistanceUnit.INCH,  72.0,  48.0, AngleUnit.DEGREES,   0.0),
            new OurPose2D(DistanceUnit.INCH,  72.0, -48.0, AngleUnit.DEGREES,   0.0),
            new OurPose2D(DistanceUnit.INCH,   0.0, -72.0, AngleUnit.DEGREES, 270.0),
            new OurPose2D(DistanceUnit.INCH, -72.0, -48.0, AngleUnit.DEGREES, 180.0)};   */

    public void runOpMode()
    {
        // init
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");

        tricep = hardwareMap.get(DcMotorEx.class, "tricep");
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");

        wrist = hardwareMap.get(Servo.class, "wrist");
        hand = hardwareMap.get(Servo.class, "hand");

        vision = new Vision(hardwareMap.get(WebcamName.class, "Webcam 1"));

        odo = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();

        xPosTimer = new ElapsedTime();
        yPosTimer = new ElapsedTime();
        hPosTimer = new ElapsedTime();

        xPosPID = new PIDController(xPosTimer);
        yPosPID = new PIDController(yPosTimer);
        hPosPID = new PIDController(hPosTimer);

        shoulderPID = new PIDController(new ElapsedTime());
        tricepPID = new PIDController(new ElapsedTime());

        xPosPID.setCoefficients(0.075, 0.0,0.0);
        yPosPID.setCoefficients(0.05, 0.0,0.0);
        hPosPID.setCoefficients(0.075, 0.0,0.0);

        shoulderPID.setCoefficients(0.075, 0.0, 0.0);

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotor[] wheels = new DcMotor[] {frontleft, frontright, backleft, backright};

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist.setPosition(0.5);
        hand.setPosition(1);

        state = AutoState.Start;

        bar = false;

        targetPosition = new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0);
        if (opModeInInit()) {
            while (opModeInInit()) {
                odometry();
            }
        }


        // play
        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                telemetry.addData("State", state);
                switch (state)
                {
                    case Start:
                    {
                        // get the robot ready to start
                        wrist.setPosition(0.25);
                        state = AutoState.RaiseArm;
                        break;
                    }
                    case RaiseArm:
                    {
                        // get the arm into position so that the specimen can be hooked
                        int targetPos = -2280;
                        int shoulderErr = targetPos - shoulder.getCurrentPosition();
//                        telemetry.addData("Shoulder Error", shoulderErr);
                        if (Math.abs(shoulderErr) < 10) {
                            shoulder.setPower(0);
                            targetPosition = new SparkFunOTOS.Pose2D(924.4, 0.0, 0.0);
                            state = AutoState.ApproachBar;
                        } else {
                            double cbrtErr = Math.signum(shoulderErr) * Math.cbrt(Math.abs(shoulderErr));
                            shoulder.setPower(shoulderPID.update(cbrtErr));
                        }
                        break;
                    }
                    case ApproachBar:
                    {
                        // drive towards the bar to hook the specimen
                        odometry();

                        // SparkFunOTOS.Pose2D position = new SparkFunOTOS.Pose2D(targetX, 0, 0.0);
                        goToPosition(targetPosition, 0.75); // state action
                        // telemetry.addData("X", position.x);

                        if (PoseMath.distance(odo.getPosition(), targetPosition) < 4.0) // state end condition
                        {
                            for (DcMotor wheel : wheels) {
                                wheel.setPower(0); // stop state action
                            }
                            state = AutoState.ExtendArm;
                            // change next state based on whether or not we want to park or touch bar
                        }
                        break;
                    }
                    case ExtendArm:
                    {
                        int tricepTarget = -600;

                        int tricepErr = tricepTarget - tricep.getCurrentPosition();
                        if (Math.abs(tricepErr) < 10)
                        {
                            tricep.setPower(0);
                            state = AutoState.ReleaseArm;
                            sleep(500);
                        }
                        else
                        {
                            double cbrtErr = Math.cbrt(tricepErr);
                            tricep.setPower(tricepPID.update(cbrtErr));
                        }

                        break;
                    }
                    case ReleaseArm:
                    {
                        hand.setPosition(0.84);

                        int targetPos = -1400;
                        telemetry.addData("Arm Pos", shoulder.getCurrentPosition());

                        int shoulderErr = targetPos - shoulder.getCurrentPosition();
                        if (Math.abs(shoulderErr) < 10)
                        {
                            shoulder.setPower(0); // stop action
                            
                            if (bar)
                            {
                                state = AutoState.TouchBar; // advance state
                                touchBarState = TouchBarSubStage.MoveBackLeft;
                                targetPosition = PoseMath.add(odo.getPosition(), new SparkFunOTOS.Pose2D(-268.0, 907.0, 0.0));
                                // prepare the robot for moving to touch bar
                            }
                            else
                            {
                                state = AutoState.Park; // advance state
                                targetPosition = PoseMath.add(odo.getPosition(), new SparkFunOTOS.Pose2D(-800.0, -1732.0, 0.0));
                                // prepare the robot for movement to corner
                            }
                        }
                        else
                        {
                            double cbrtErr = Math.cbrt(shoulderErr);
                            shoulder.setPower(shoulderPID.update(cbrtErr));
                        }
                        break;
                    }
                    case Park:
                    {
                        // park in the space or touch the bar
                        odometry();
                        goToPosition(targetPosition, 0.75);

                        if (PoseMath.distance(odo.getPosition(), targetPosition) < 4.0)
                        {
                            for (DcMotor wheel : wheels)
                            {
                                wheel.setPower(0);
                            }
                            state = AutoState.End;
                        }

                        break;
                    }
                    case TouchBar:
                    {
                        // move the robot to touch the bar
                        odometry();

                        switch (touchBarState)
                        {
                            case MoveBackLeft:
                            {
                                goToPosition(targetPosition, 0.75);

                                if (PoseMath.distance(odo.getPosition(), targetPosition) < 4.0)
                                {
                                    for (DcMotor wheel : wheels)
                                    {
                                        wheel.setPower(0);
                                    }
                                    touchBarState = TouchBarSubStage.MoveForward;
                                    targetPosition = PoseMath.add(odo.getPosition(), new SparkFunOTOS.Pose2D(635.0, 0.0, 0.0));
                                }
                                break;   
                            }
                            case MoveForward:
                            {
                                goToPosition(targetPosition, 0.75);

                                if (PoseMath.distance(odo.getPosition(), targetPosition) < 4.0)
                                {
                                    for (DcMotor wheel : wheels)
                                    {
                                        wheel.setPower(0);
                                    }
                                    touchBarState = TouchBarSubStage.MoveRight;
                                    targetPosition = PoseMath.add(odo.getPosition(), new SparkFunOTOS.Pose2D(0.0, 528.0, 0.0));
                                }                                
                                break;
                            }
                            case MoveRight:
                            {
                                goToPosition(targetPosition, 0.75);

                                if (PoseMath.distance(odo.getPosition(), targetPosition) < 4.0)
                                {
                                    for (DcMotor wheel : wheels)
                                    {
                                        wheel.setPower(0);
                                    }
                                    touchBarState = TouchBarSubStage.MoveArm;
                                }
                                break;
                            }
                            case MoveArm:
                            {
                                int armTarget = -1000;
                                int shoulderErr = shoulder.getCurrentPosition() - armTarget;

                                if (Math.abs(shoulderErr) < 10) // end condition
                                {
                                    shoulder.setPower(0); // stop action
                                    state = AutoState.End;
                                }
                                else
                                {
                                    double cbrtErr = Math.cbrt(shoulderErr);
                                    shoulder.setPower(shoulderPID.update(cbrtErr)); // sub-state action
                                }
                                break;
                            }
                        }
                        break;
                    }
                    case End:
                    {
                        // end behavior
                        break;
                    }
                }
                telemetry.update();
            }

        }
    }

    public void odometry()
    {
        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
        */

        SparkFunOTOS.Pose2D pos = odo.getPosition();
        // APRIL TAG POSITION CORRECTION PROTOCOL
//        AprilTagDetection tag = null;
//        AprilTagDetection[] detections = vision.detect();
//        if (detections.length > 0) {
//            tag = detections[0];
//        }
//        if (tag != null)
//        {
//            double robotX = tag.robotPose.getPosition().x;
//            double robotY = tag.robotPose.getPosition().y;
//            double robotH = tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
//
//            SparkFunOTOS.Pose2D position = new SparkFunOTOS.Pose2D(robotY, robotX, robotH);
//            odo.setPosition(position);
//        }

        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.x, pos.y, pos.h);
        telemetry.addData("Position", data);
    }

    public void goToPosition(SparkFunOTOS.Pose2D position)
    {
        goToPosition(position, 1.0);
    }

    public void goToPosition(SparkFunOTOS.Pose2D position, double powerCoeff){

        SparkFunOTOS.Pose2D currentPos = odo.getPosition();

        double currentXPos = currentPos.x;
        double currentYPos = currentPos.y;
        double currentHPos = currentPos.h;

        double xPos = position.x;
        double yPos = position.y;
        double hPos = position.h;

        double xErr = xPos - currentXPos;
        double yErr = yPos - currentYPos;
        double hErr = hPos - currentHPos;

        double cbrtXError = Math.cbrt(xErr);
        double cbrtYError = Math.cbrt(yErr);
        double cbrtHError = Math.cbrt(hErr);

        telemetry.addLine("xErr: " + xErr + ", cbrtXErr: " + cbrtXError);
        telemetry.addLine("yErr: " + yErr + ", cbrtYErr: " + cbrtYError);
        telemetry.addLine("hErr: " + hErr + ", cbrtHErr: " + cbrtHError);

        double xPower = xPosPID.update(cbrtXError);
        double yPower = yPosPID.update(cbrtYError);
        double hPower = hPosPID.update(cbrtHError);

        double theta = currentHPos * (Math.PI / 180.0);
        double forward  = (yPower * Math.sin(theta) + xPower * Math.cos(theta));
        double sideways = (yPower * Math.cos(theta) - xPower * Math.sin(theta));

        double FL = forward + sideways + hPower;
        double FR = forward - sideways - hPower;
        double BL = forward + sideways - hPower;
        double BR = forward - sideways + hPower;

        frontleft.setPower(FL * powerCoeff);
        frontright.setPower(FR * powerCoeff);
        backleft.setPower(BL * powerCoeff);
        backright.setPower(BR * powerCoeff);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        odo.setLinearUnit(DistanceUnit.MM);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        odo.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(8, -120, 0);
        odo.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        odo.setLinearScalar(2438.4 / 2307.7393);
        odo.setAngularScalar(3600.0 / 3621.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        odo.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        odo.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        odo.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        odo.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}
