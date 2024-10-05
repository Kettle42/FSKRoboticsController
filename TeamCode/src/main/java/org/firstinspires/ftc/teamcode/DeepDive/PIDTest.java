package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

        rangepid.setCoefficients(0.2, 0.0, 0.0);
        bearingpid.setCoefficients(0.2, 0.0, 0.0);
        yawpid.setCoefficients(0.2, 0.0, 0.0);
//        pid.setCoefficients(0.24, 0.48 / 2.0, 2.4 / 40.0);

        double targetRange = 10;
        double targetBearing = 0;
        double targetYaw = 0;

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                telemetry.clear();
//              double x = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
                double x = 0;
                double l = 0;

                AprilTagDetection tag = vision.tryGetTag(11);
                double rangeoutput = 0;
                double bearingoutput = 0;
                double yawoutput = 0;
                if (tag != null)
                {
                    double range = tag.ftcPose.range;
                    double bearing = tag.ftcPose.bearing;
                    double yaw = tag.ftcPose.yaw;

                    double rangeToTarget = range - targetRange;

                    telemetry.addData("range to target", rangeToTarget);
                    telemetry.addData("range to target", bearing);
                    telemetry.addData("yaw to target", yaw);

                    rangeoutput = rangepid.update(rangeToTarget, 0);
                    bearingoutput = bearingpid.update(bearing, targetBearing);
                    yawoutput = bearingpid.update(yaw, targetYaw);

                    telemetry.addData("pid coefficients", String.format("(%s, %s, %s)", rangepid.kp, rangepid.ki, rangepid.kd));
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

                double[] powers = new double[] {FL, FR, BL, BR};
                double maxAbsolute = maxAbsolute(powers);

                for (int i = 0; i < powers.length; i++) {
                    powers[i] *= 0.25 / maxAbsolute;
                }

                frontleft.setPower(powers[0]);
                frontright.setPower(powers[1]);
                backleft.setPower(powers[2]);
                backright.setPower(powers[3]);

                telemetry.update();
                // here is a comment that will show up in the Android Studio version after I commit, push, and pull
            }
        }
    }

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
