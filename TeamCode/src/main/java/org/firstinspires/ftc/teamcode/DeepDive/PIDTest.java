package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "PIDTest")
public class PIDTest extends LinearOpMode
{
    public DcMotorEx frontleft;
    public DcMotorEx frontright;
    public DcMotorEx backleft;
    public DcMotorEx backright;
    public DcMotorEx[] wheels;
    private ElapsedTime rangetimer;
    private ElapsedTime bearingtimer;
    private ElapsedTime yawtimer;
    PIDController rangepid;
    PIDController bearingpid;
    PIDController yawpid;
    Vision vision;
    static Pose2D[] tagXYH = new Pose2D[] {new Pose2D(DistanceUnit.INCH, -72.0,  48.0, AngleUnit.DEGREES, 180.0),
                                           new Pose2D(DistanceUnit.INCH,   0.0,  72.0, AngleUnit.DEGREES,  90.0),
                                           new Pose2D(DistanceUnit.INCH,  72.0,  48.0, AngleUnit.DEGREES,   0.0),
                                           new Pose2D(DistanceUnit.INCH,  72.0, -48.0, AngleUnit.DEGREES,   0.0),
                                           new Pose2D(DistanceUnit.INCH,   0.0, -72.0, AngleUnit.DEGREES, 270.0),
                                           new Pose2D(DistanceUnit.INCH, -72.0, -48.0, AngleUnit.DEGREES, 180.0)};
    PIDController[] pids;

    public enum RBY {
        RANGE,
        BEARING,
        YAW
    }

    public enum PID {
        PROPORTIONAL,
        INTEGRAL,
        DERIVATIVE
    }


    @Override
    public void runOpMode()
    {
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        vision = new Vision(hardwareMap.get(WebcamName.class, "Webcam 1"));

        wheels = new DcMotorEx[] {frontleft, frontright, backleft, backright};

        rangetimer = new ElapsedTime();
        bearingtimer = new ElapsedTime();
        yawtimer = new ElapsedTime();
        rangepid = new PIDController(rangetimer);
        bearingpid = new PIDController(bearingtimer);
        yawpid = new PIDController(yawtimer);

        for (DcMotorEx wheel : wheels)
        {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);

        rangepid.setCoefficients(0.0, 0.0, 0.0);
        bearingpid.setCoefficients(0.0, 0.0, 0.0);
        yawpid.setCoefficients(0.0, 0.0, 0.0);

        pids = new PIDController[] { rangepid, bearingpid, yawpid };
//        pid.setCoefficients(0.24, 0.48 / 2.0, 2.4 / 40.0);

        double targetRange = 10;
        double targetBearing = 0;
        double targetYaw = 0;

        double maxAbsolute = 0.0;
        int count = 0;
        int rbycount = 0;
        int pidcount = 0;
        boolean moving = false;
        double offset = 0.01;

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive()) {
                telemetry.clear();
                if (gamepad1.left_bumper) {
                    boolean saved = moving;
                    while(gamepad1.left_bumper) {
                        moving = !(saved);
                    }
                }
                if (moving)
                {
                    AprilTagDetection tag = null;
                    AprilTagDetection[] detections = vision.detect();
                    Pose2D tagPose2D = tagXYH[tag.id - 11];
                    if (detections.length > 0)
                    {
                        tag = detections[0];
                    }
                    double rangeoutput = 0;
                    double bearingoutput = 0;
                    double yawoutput = 0;
                    if (tag != null) {
                        double range = tag.ftcPose.range;
                        double bearing = tag.ftcPose.bearing;
                        double yaw = tag.ftcPose.yaw;

                        double x = tag.ftcPose.x;
                        double y = tag.ftcPose.y;

                        double tagHeading = tagPose2D.getHeading(AngleUnit.DEGREES);

                        Position pos = tag.robotPose.getPosition();
                        YawPitchRollAngles orient = tag.robotPose.getOrientation();

                        double robotHeading = (90 - yaw) + tagHeading;
                        double robotX = pos.x;
                        double robotY = pos.y;

                        double rangeToTarget = range - targetRange;

                        /*telemetry.addData("range to target", rangeToTarget);
                        telemetry.addData("bearing to target", bearing);
                        telemetry.addData("yaw to target", yaw);
                        telemetry.addData("x to target", x);
                        telemetry.addData("y to target", y);*/

                        telemetry.addLine("Robots X Position on Field: " + robotX);
                        telemetry.addLine("Robots Y Position on Field: " + robotY);
                        telemetry.addLine("Robots Heading: " + robotHeading);

                        rangeoutput = rangepid.update(rangeToTarget, 0);
                        bearingoutput = bearingpid.update(bearing, targetBearing);
                        yawoutput = yawpid.update(yaw, targetYaw);

                        //telemetry.addData("pid coefficients", String.format("(%s, %s, %s)", rangepid.kp, rangepid.ki, rangepid.kd));
                    }
                    //              double l = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2))

                    telemetry.addLine(String.valueOf(tag != null));

                    double FL = 0;
                    double FR = 0;
                    double BL = 0;
                    double BR = 0;

                    FL += rangeoutput + -bearingoutput + yawoutput;
                    FR += rangeoutput + bearingoutput + -yawoutput;
                    BL += rangeoutput + -bearingoutput + -yawoutput;
                    BR += rangeoutput + bearingoutput + yawoutput;

                    //telemetry.addLine("FL: " + FL + " FR: " + FR + " BL: " + BL + " BR: " + BR);

                    double[] powers = new double[]{FL, FR, BL, BR};
                    double localMax = maxAbsolute(powers);
                    if (localMax > maxAbsolute) {
                        maxAbsolute = localMax;
                    }
                    //telemetry.addLine("MAX ABSOLUTE: " + maxAbsolute);

                    //maxAbsolute = maxAbsolute(powers);

                    /*for (int i = 0; i < powers.length; i++) {
                        powers[i] *= 0.6 / maxAbsolute;
                    }*/

                    frontleft.setVelocity(powers[0]);
                    frontright.setVelocity(powers[1]);
                    backleft.setVelocity(powers[2]);
                    backright.setVelocity(powers[3]);

//                    count += 1;
                    telemetry.update();
                    // here is a comment that will show up in the Android Studio version after I commit, push, and pull
                } else { // not moving
                    frontleft.setPower(0);
                    frontright.setPower(0);
                    backleft.setPower(0);
                    backright.setPower(0);
                    //telemetry.addLine("not moving");
                    //telemetry.addLine();
//                    maxAbsolute = 0.0;

                    for (PIDController pid : pids)
                    {
                        pid.integralSum = 0.00;
                    }

                    if (gamepad1.square) {
                        int prev = rbycount;
                        while (gamepad1.square) {
                            rbycount = prev + 1;
                        }
                    }

                    if (gamepad1.circle) {
                        int prev = pidcount;
                        while (gamepad1.circle) {
                            pidcount = prev + 1;
                        }
                    }

                    RBY rbyval = RBY.values()[rbycount % 3];
                    PID pidval = PID.values()[pidcount % 3];

                    PIDController pid;

                    switch(rbyval) {
                        case RANGE:
                            pid = rangepid;
                            telemetry.addLine("RANGE PID IS BEING ALTERED");
                            break;
                        case BEARING:
                            pid = bearingpid;
                            telemetry.addLine("BEARING PID IS BEING ALTERED");
                            break;
                        case YAW:
                            pid = yawpid;
                            telemetry.addLine("YAW PID IS BEING ALTERED");
                            break;
                        default:
                            ElapsedTime t = new ElapsedTime();
                            pid = new PIDController(t);
                            telemetry.addLine("PID IS NULL");
                            break;
                    }
                    telemetry.addLine("PRESS SQUARE/X TO CYCLE PIDS");
                    telemetry.addLine();
                    int index;
                    switch(pidval) {
                        case PROPORTIONAL:
                            index = 0;
                            telemetry.addLine("CHANGING THE PROPORTIONAL COEFFICIENT");
                            break;
                        case INTEGRAL:
                            index = 1;
                            telemetry.addLine("CHANGING THE INTEGRAL COEFFICIENT");
                            break;
                        case DERIVATIVE:
                            index = 2;
                            telemetry.addLine("CHANGING THE DERIVATIVE COEFFICIENT");
                            break;
                        default:
                            index = 0;
                            telemetry.addLine("PIDVAL OUT OF RANGE");
                            break;
                    }
                    telemetry.addLine("PRESS CIRCLE/B CYCLE COEFFICIENTS");
                    telemetry.addLine();

                    telemetry.addLine("DPAD UP/DOWN TO ADD/SUBTRACT OFFSET");
                    telemetry.addLine("DPAD LEFT/RIGHT TO 10x/0.1x THE OFFSET");
                    telemetry.addLine("PRESS TRIANGLE/Y TO ZERO SELECTED");
                    telemetry.addLine("OFFSET IS: " + offset);

                    double[] coeffs = pid.getCoefficients();
                    if (gamepad1.dpad_up) {
                        double prev = coeffs[index];
                        while (gamepad1.dpad_up){
                            coeffs[index] = prev + offset;
                        }
                    }
                    if (gamepad1.dpad_down) {
                        double prev = coeffs[index];
                        while (gamepad1.dpad_down){
                            coeffs[index] = prev - offset;
                        }
                    }

                    if (gamepad1.dpad_left) {
                        double prev = offset;
                        while (gamepad1.dpad_left){
                            offset = prev * 10.0;
                        }
                    }

                    if (gamepad1.dpad_right) {
                        double prev = offset;
                        while (gamepad1.dpad_right){
                            offset = prev / 10.0;
                        }
                    }

                    if (gamepad1.triangle)
                    {
                        while (gamepad1.triangle)
                        {
                            coeffs[index] = 0.00;
                        }
                    }

                    pid.setCoefficients(coeffs[0], coeffs[1], coeffs[2]);

                    telemetry.addLine(pid.toString());
                    telemetry.update();
                } // end not moving branch
            } // end while
        } // end if
        RobotLog.a("End of operation values!");
        RobotLog.a("Range PID: " + rangepid.toString());
        RobotLog.a("Bearing PID: " + bearingpid.toString());
        RobotLog.a("Yaw PID: " + yawpid.toString());

    } // end method

    public static double clamp(double n, double lo, double hi)
    {
        return Math.min(Math.max(n, lo), hi);
    }

    public static double sign(double n)
    {
        if (n > 0) return 1;
        if (n < 0) return -1;
        return 0;
    }

    public static double maxAbsolute(double[] vals) {

        double max = 0;
        for (double val : vals) {
            if (Math.abs(val) > max) {
                max = Math.abs(val);
            }
        }

        return max;
    }
}
