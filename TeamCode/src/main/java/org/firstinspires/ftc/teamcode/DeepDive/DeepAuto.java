package org.firstinspires.ftc.teamcode.DeepDive;

import android.util.Size;

import java.util.*;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name = "Deep Auto v1.3.5")
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

    // static variables are able to be changed by the dashboard
    public static int shoulderClipTarget = 2500;
    public static int shoulderReleaseTarget = 1800;
    public static int motorErrorMax = 5;
    public static double posErrorMax = 4.0;
    public static double forwardFromStart = 800.0;
    public static double wheelPower = 0.75;


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

        xPosPID.setCoefficients(0.085, 0.0,0.0);
        yPosPID.setCoefficients(0.085, 0.0,0.0);
        hPosPID.setCoefficients(0.085, 0.0,0.0);

        shoulderPID.setCoefficients(0.125, 0.0, 0.0);

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotor[] wheels = new DcMotor[] {frontleft, frontright, backleft, backright};

//        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
//        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist.setPosition(HandValues.WristMode.Folded.position);
        hand.setPosition(HandValues.ClawMode.Clippy.position);

        state = AutoState.Start;

        bar = false;

        double wristPosForShoulderTarget = DeepDrive.getWristPosFromAngle(-(DeepDrive.getShoulderAngleFromPos(shoulderClipTarget) - 45.0)) + 0.1;
        telemetry.addData("wristPos", wristPosForShoulderTarget + "");
        telemetry.update();

        targetPosition = new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0);
//        if (opModeInInit()) {
//            while (opModeInInit()) {
//                odometry();
//            }
//        }

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
                        wrist.setPosition(wristPosForShoulderTarget);
                        state = AutoState.RaiseArm;
                        break;
                    }
                    case RaiseArm:
                    {
                        // get the arm into position so that the specimen can be hooked
                        int shoulderErr = shoulderClipTarget - shoulder.getCurrentPosition();
                        telemetry.addData("Shoulder Error", shoulderErr);
                        if (Math.abs(shoulderErr) < motorErrorMax) {
                            shoulder.setPower(0);
                            targetPosition = new SparkFunOTOS.Pose2D(forwardFromStart, 0.0, 0.0);
                            state = AutoState.ApproachBar;
                        } else {
                            double cbrtErr = Math.cbrt(shoulderErr);
                            shoulder.setPower(shoulderPID.update(cbrtErr));
                        }
                        break;
                    }
                    case ApproachBar:
                    {
                        // drive towards the bar to hook the specimen
                        odometry();
                        wrist.setPosition(wristPosForShoulderTarget);
                        hand.setPosition(HandValues.ClawMode.Clippy.position);
                        // SparkFunOTOS.Pose2D position = new SparkFunOTOS.Pose2D(targetX, 0, 0.0);
                        goToPosition(targetPosition, wheelPower); // state action
                        // telemetry.addData("X", position.x);

                        if (PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax) // state end condition
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
//                        int tricepTarget = -600;
//
//                        int tricepErr = tricepTarget - tricep.getCurrentPosition();
//                        if (Math.abs(tricepErr) < 10)
//                        {
//                            tricep.setPower(0);
                            state = AutoState.ReleaseArm;
//                            sleep(500);
//                        }
//                        else
//                        {
//                            double cbrtErr = Math.cbrt(tricepErr);
//                            tricep.setPower(tricepPID.update(cbrtErr));
//                        }
//
                        break;
                    }
                    case ReleaseArm:
                    {
                        hand.setPosition(HandValues.ClawMode.Clippy.position + HandValues.ClawMode.Clippy.openOffset);

                        telemetry.addData("Arm Pos", shoulder.getCurrentPosition());

                        int shoulderErr = shoulderReleaseTarget - shoulder.getCurrentPosition();
                        if (Math.abs(shoulderErr) < motorErrorMax)
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

                        if (PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax)
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

                                if (PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax)
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

                                if (PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax)
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

                                if (PoseMath.distance(odo.getPosition(), targetPosition) < posErrorMax)
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

                                if (Math.abs(shoulderErr) < motorErrorMax) // end condition
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
        double forward  = (xPower * Math.cos(theta) - yPower * Math.sin(theta));
        double sideways = (xPower * Math.sin(theta) + yPower * Math.cos(theta));

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
//        odo.setLinearScalar((2438.4) / (2373.6));
//        odo.setAngularScalar((3600.0) / (3641.748));

        odo.calibrateImu();
        odo.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        odo.setPosition(currentPosition);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.update();
    }
}
