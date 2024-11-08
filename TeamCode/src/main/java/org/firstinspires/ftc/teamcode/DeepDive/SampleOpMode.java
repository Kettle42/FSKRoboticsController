package org.firstinspires.ftc.teamcode.DeepDive;

import java.util.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

// import org.firstinspires.ftc.teamcode.DeepDive.XyhVector;

@TeleOp(name = "Deep Dive v1.0.0")

public class SampleOpMode extends LinearOpMode
{
    private DcMotor frontright;
    private DcMotor frontleft;
    private DcMotor backright;
    private DcMotor backleft;

    GoBildaPinpointDriver odo;

    private static final float deadzone = 0.25f;

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

    Vision vision;

    static Pose2D[] tagXYH = new Pose2D[] { new Pose2D(DistanceUnit.INCH, -72.0,  48.0, AngleUnit.DEGREES, 180.0),
                                            new Pose2D(DistanceUnit.INCH,   0.0,  72.0, AngleUnit.DEGREES,  90.0),
                                            new Pose2D(DistanceUnit.INCH,  72.0,  48.0, AngleUnit.DEGREES,   0.0),
                                            new Pose2D(DistanceUnit.INCH,  72.0, -48.0, AngleUnit.DEGREES,   0.0),
                                            new Pose2D(DistanceUnit.INCH,   0.0, -72.0, AngleUnit.DEGREES, 270.0),
                                            new Pose2D(DistanceUnit.INCH, -72.0, -48.0, AngleUnit.DEGREES, 180.0)};

    public void runOpMode()
    {
        // init
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");

        vision = new Vision(hardwareMap.get(WebcamName.class, "Webcam 1"));

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"imu");
        odo.setOffsets(xOffset, yOffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        xPosTimer = new ElapsedTime();
        yPosTimer = new ElapsedTime();
        hPosTimer = new ElapsedTime();

        xPosPID = new PIDController(xPosTimer);
        yPosPID = new PIDController(yPosTimer);
        hPosPID = new PIDController(hPosTimer);

        xPosPID.setCoefficients(0.05 , 0.0,0.0);
        yPosPID.setCoefficients(0.05, 0.0,0.0);
        hPosPID.setCoefficients(0.05, 0.0,0.0);

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);

        // play
        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                /*float power = (1 - gamepad1.right_trigger) * 0.5f;

                float lx = gamepad1.left_stick_x;
                float ly = gamepad1.left_stick_y;
                float rx = gamepad1.right_stick_x;
                if (Math.abs(lx) < deadzone) lx = 0;
                if (Math.abs(ly) < deadzone) ly = 0;
                if (Math.abs(rx) < deadzone) rx = 0;

                frontleft.setPower((-lx + ly - rx) * power);
                frontright.setPower((lx + ly + rx) * power);
                backright.setPower((-lx + ly + rx) * power);
                backleft.setPower((lx + ly - rx) * power);*/

                odometry();
                Pose2D position = new Pose2D(DistanceUnit.MM, 2000, 500, AngleUnit.DEGREES, 90);
                goToPosition(position);

                telemetry.update();
            }
        }
    }

    public void odometry()
    {

        odo.bulkUpdate();

        /*
        This code prints the loop frequency of the REV Control Hub. This frequency is effected
        by I2C reads/writes. So it's good to keep an eye on. This code calculates the amount
        of time each cycle takes and finds the frequency (number of updates per second) from
        that cycle time.
         */
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;


        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
         */
        Pose2D pos = odo.getPosition();

        AprilTagDetection tag = null;
        AprilTagDetection[] detections = vision.detect();
        if (detections.length > 0) {
            tag = detections[0];
        }
        if (tag != null) {
            double yaw = tag.ftcPose.yaw;
            double bearing = tag.ftcPose.bearing;

            telemetry.addLine("yaw          : " + yaw);
            telemetry.addLine("bearing      : " + bearing);
            telemetry.addLine("bearing - yaw: " + (bearing - yaw));

            Pose2D tagPose2D = tagXYH[tag.id - 11];

            double robotX = tag.robotPose.getPosition().x;
            double robotY = tag.robotPose.getPosition().y;
            double robotH = tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

            Pose2D position = new Pose2D(DistanceUnit.MM, robotY, robotX, AngleUnit.DEGREES, robotH);
            odo.setPosition(position);
        }

        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        /*
        gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
         */
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        telemetry.addData("X Encoder:", odo.getEncoderX()); //gets the raw data from the X encoder
        telemetry.addData("Y Encoder:",odo.getEncoderY()); //gets the raw data from the Y encoder
//        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        /*
        Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
        READY: the device is working as normal
        CALIBRATING: the device is calibrating and outputs are put on hold
        NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
        FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
        FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
        FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
        */
        telemetry.addData("Status", odo.getDeviceStatus());

        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

    }

    public void goToPosition(Pose2D position){

        Pose2D currentPos = odo.getPosition();

        double currentXPos = currentPos.getX(DistanceUnit.MM);
        double currentYPos = currentPos.getY(DistanceUnit.MM);
        double currentHPos = currentPos.getHeading(AngleUnit.DEGREES);

        double xPos = position.getX(DistanceUnit.MM);
        double yPos = position.getY(DistanceUnit.MM);
        double hPos = position.getHeading(AngleUnit.DEGREES);

        double xErr = xPos - currentXPos;
        double yErr = yPos - currentYPos;
        double hErr = hPos - currentHPos;

        double cbrtXError = Math.signum(xErr) * (Math.pow(Math.abs(xErr), 1.0/3.0));
        double cbrtYError = Math.signum(yErr) * (Math.pow(Math.abs(yErr), 1.0/3.0));
        double cbrtHError = Math.signum(hErr) * (Math.pow(Math.abs(hErr), 1.0/3.0));

        telemetry.addLine("xErr: " + xErr + ", cbrtXErr: " + cbrtXError);
        telemetry.addLine("yErr: " + yErr + ", cbrtYErr: " + cbrtYError);
        telemetry.addLine("hErr: " + hErr + ", cbrtHErr: " + cbrtHError);

        double xPower = xPosPID.update(cbrtXError);
        double yPower = yPosPID.update(cbrtYError);
        double hPower = hPosPID.update(cbrtHError);

        double theta = currentHPos * (Math.PI / 180.0);
        double forward  = yPower * Math.sin(theta) + xPower * Math.cos(theta);
        double sideways = yPower * Math.cos(theta) - xPower * Math.sin(theta);

        double FL = -forward + sideways + hPower;
        double FR = -forward - sideways - hPower;
        double BL = -forward - sideways + hPower;
        double BR = -forward + sideways - hPower;

        /*frontleft.setPower(FL);
        frontright.setPower(FR);
        backleft.setPower(BL);
        backright.setPower(BR);*/
    }
}