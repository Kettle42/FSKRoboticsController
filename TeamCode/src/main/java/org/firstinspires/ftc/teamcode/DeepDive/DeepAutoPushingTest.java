package org.firstinspires.ftc.teamcode.DeepDive;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KettleLibrary.PIDController;
import org.firstinspires.ftc.teamcode.KettleLibrary.Vision;

import java.util.Locale;

// import org.firstinspires.ftc.teamcode.KettleLibrary.XyhVector;

@Config
@Autonomous(name = "Deep Auto Pushing Test")
public class DeepAutoPushingTest extends LinearOpMode
{
    public enum AutoState
    {
        MoveBackRight,
        TowardsSamples,
        BehindSampleOne,
        Turning,
        PushingSamples,
        Park,
        End;

        public AutoState next()
        {
            if (this == End) return End;
            return AutoState.values()[(this.ordinal() + 1)];
        }
    }

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

        public static SparkFunOTOS.Pose2D constructAdjusted(double x, double y, double h)
        {
            return new SparkFunOTOS.Pose2D(y, -x, h - 90);
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

    SparkFunOTOS.Pose2D targetPosition;
    SparkFunOTOS.Pose2D[] pushingPositions;

    /*static OurPose2D[] tagXYH = new OurPose2D[] { new OurPose2D(DistanceUnit.INCH, -72.0,  48.0, AngleUnit.DEGREES, 180.0),
            new OurPose2D(DistanceUnit.INCH,   0.0,  72.0, AngleUnit.DEGREES,  90.0),
            new OurPose2D(DistanceUnit.INCH,  72.0,  48.0, AngleUnit.DEGREES,   0.0),
            new OurPose2D(DistanceUnit.INCH,  72.0, -48.0, AngleUnit.DEGREES,   0.0),
            new OurPose2D(DistanceUnit.INCH,   0.0, -72.0, AngleUnit.DEGREES, 270.0),
            new OurPose2D(DistanceUnit.INCH, -72.0, -48.0, AngleUnit.DEGREES, 180.0)};   */

    // static variables are able to be changed by the dashboard
    public static int shoulderClipTarget = 2500;
    public static int shoulderReleaseTarget = 1800;
    public static int motorErrorMax = 5;
    public static double posErrorMax = 4.0;
    public static double angleErrorMax = 5.0;
    public static double forwardFromStart = 815.0;

    public static double behindSamples = 500;
    public static double inZone = -750;
    public static double firstSample = -1100;
    public static double secondSample = -1300;
    public static double thirdSample = -1450;
    public static double rightAtStart = -800;

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

        Vision.CameraOffset cameraOffset = new Vision.CameraOffset(0.0, 153.0, 59.0, 0.0, -90.0, -90.0);
        vision = new Vision(hardwareMap.get(WebcamName.class, "Webcam 1"), Vision.LensIntrinsics.LogitechC270, cameraOffset, new Size(640, 480));

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

        xPosPID.setCoefficients(0.095, 0.0,0.0);
        yPosPID.setCoefficients(0.095, 0.0,0.0);
        hPosPID.setCoefficients(0.095, 0.0,0.0);

        shoulderPID.setCoefficients(0.15, 0.0, 0.0);

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        state = AutoState.MoveBackRight;

        DcMotor[] wheels = new DcMotor[] {frontleft, frontright, backleft, backright};

//        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
//        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist.setPosition(HandValues.WristMode.Folded.position);
        hand.setPosition(HandValues.ClawMode.Clippy.position);
        double wristPosForShoulderTarget = DeepDrive.getWristPosFromAngle(-(DeepDrive.getShoulderAngleFromPos(shoulderClipTarget) - 45.0)) + 0.1;
        telemetry.addData("wristPos", wristPosForShoulderTarget + "");
        telemetry.update();

        targetPosition = PoseMath.constructAdjusted(-304.8, rightAtStart, 0.0);
//        if (opModeInInit()) {
//            while (opModeInInit()) {
//                odometry();
//            }
//        }

        int pushingIterations = 0;

        pushingPositions = new SparkFunOTOS.Pose2D[]
                {
                        PoseMath.constructAdjusted(inZone, firstSample, 180),
                        PoseMath.constructAdjusted(behindSamples, firstSample, 180),
                        PoseMath.constructAdjusted(behindSamples, secondSample, 180),
                        PoseMath.constructAdjusted(inZone, secondSample, 180),
                        PoseMath.constructAdjusted(behindSamples, secondSample, 180),
                        PoseMath.constructAdjusted(behindSamples, thirdSample, 180),
                        PoseMath.constructAdjusted(inZone, thirdSample, 180)
                };


        // play
        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                telemetry.addData("State", state);
                telemetry.addData("time", timer.seconds());
                switch (state)
                {
                    case MoveBackRight:
                    {
                        // target position has been set to not 0,0,0
                        goToPosition(targetPosition, 0.5);
                        if (DeepAutoPushingTest.PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax) // state end condition
                        {
                            for (DcMotor wheel : wheels) {
                                wheel.setPower(0); // stop state action
                            }
                            targetPosition = PoseMath.constructAdjusted(behindSamples, rightAtStart, 180);
                            state = AutoState.TowardsSamples;
                            // change next state based on whether or not we want to park or touch bar
                        }
                        break;
                    }
                    case TowardsSamples:
                    {
                        goToPosition(targetPosition, 0.5);
                        if (DeepAutoPushingTest.PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax) // state end condition
                        {
                            for (DcMotor wheel : wheels) {
                                wheel.setPower(0); // stop state action
                            }
                            targetPosition = PoseMath.constructAdjusted(behindSamples, firstSample, 180);
                            state = AutoState.BehindSampleOne;
                            // change next state based on whether or not we want to park or touch bar
                        }
                        break;
                    }
                    case BehindSampleOne:
                    {
                        goToPosition(targetPosition, 0.5);
                        if (DeepAutoPushingTest.PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax) // state end condition
                        {
                            for (DcMotor wheel : wheels) {
                                wheel.setPower(0); // stop state action
                            }
                            state = AutoState.PushingSamples;
                            // change next state based on whether or not we want to park or touch bar{
                        }
                        break;
                    }
//                    case Turning:
//                    {
//                        SparkFunOTOS.Pose2D newPos =
//                                new SparkFunOTOS.Pose2D(targetPosition.x, targetPosition.y, 90);
//
//                        goToPosition(newPos);
//
//                        if (Math.abs(odo.getPosition().h - targetPosition.h) < angleErrorMax)
//                        {
//                            for (DcMotor wheel : wheels)
//                            {
//                                wheel.setPower(0);
//                            }
//
//                            state = AutoState.PushingSamples;
//                        }
//
//                        break;
//                    }
                    case PushingSamples:
                    {
                        targetPosition = pushingPositions[pushingIterations];
                        goToPosition(targetPosition, 0.5);
                        if (DeepAutoPushingTest.PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax) // state end condition
                        {
                            for (DcMotor wheel : wheels) {
                                wheel.setPower(0); // stop state action
                            }
                            pushingIterations++;
                        }
                        if (pushingIterations == pushingPositions.length) {
                            state = AutoState.Park;
                        }

                        break;
                    }
                    case Park:
                    {
                        state = AutoState.End;
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

    public void openClaw(HandValues.ClawMode direction)
    {
        hand.setPosition(direction.position + direction.openOffset);
    }

    public void closeClaw(HandValues.ClawMode direction)
    {
        hand.setPosition(direction.position);
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

        /*telemetry.addLine("xErr: " + xErr + ", cbrtXErr: " + cbrtXError);
        telemetry.addLine("yErr: " + yErr + ", cbrtYErr: " + cbrtYError);
        telemetry.addLine("hErr: " + hErr + ", cbrtHErr: " + cbrtHError);*/

        double xPower = xPosPID.update(cbrtXError);
        double yPower = yPosPID.update(cbrtYError);
        double hPower = hPosPID.update(cbrtHError);

        double theta = currentHPos * (Math.PI / 180.0);
        double forward  = (yPower * Math.sin(theta) + xPower * Math.cos(theta));
        double sideways = (yPower * Math.cos(theta) - xPower * Math.sin(theta));

        double FL = forward - sideways - hPower;
        double FR = forward + sideways + hPower;
        double BL = forward + sideways - hPower;
        double BR = forward - sideways + hPower;

        /*telemetry.addLine("Forward: " + forward + "\nSideways: " + sideways);
        telemetry.addLine("FL: " + FL);
        telemetry.addLine("FR: " + FR);
        telemetry.addLine("BL: " + BL);
        telemetry.addLine("BR: " + BR);*/

        frontleft.setPower(FL * powerCoeff);
        frontright.setPower(FR * powerCoeff);
        backleft.setPower(BL * powerCoeff);
        backright.setPower(BR * powerCoeff);
        //telemetry.update();
    }

    private void configureOtos()
    {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        odo.setLinearUnit(DistanceUnit.MM);
        odo.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(153.5, 0.0, 0.0);
        odo.setOffset(offset);

        odo.setLinearScalar((2438.4) / (2448.425));
        odo.setAngularScalar((4680.0) / (4701.7474));

        odo.calibrateImu();
        odo.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, -90);
        odo.setPosition(currentPosition);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.update();
    }

    public double getTurningAngle(double angleDegrees)
    {
        double h = Math.toRadians(odo.getPosition().h);
        angleDegrees = Math.toRadians(angleDegrees);
        return Math.signum(h) * Math.acos( Math.cos(angleDegrees) * Math.cos(h) + Math.sin(angleDegrees) * Math.sin(h) );
    }
}
