package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Angle Drive")
public class AngleDrive extends LinearOpMode
{
    public DcMotorEx frontleft;
    public DcMotorEx frontright;
    public DcMotorEx backleft;
    public DcMotorEx backright;
    public DcMotorEx[] wheels;
    private ElapsedTime timer;
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

        timer = new ElapsedTime();
        rangepid = new PIDController(timer);

        for (DcMotorEx wheel : wheels)
        {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);

        rangepid.setCoefficients(0.3, 0.0, 0.0);
//        pid.setCoefficients(0.24, 0.48 / 2.0, 2.4 / 40.0);

        double targetRange = 10;

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
//                double x = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
                double x = 0;
                double l = 0;

                AprilTagDetection tag = vision.tryGetTag(11);
                double output = 0;
                if (tag != null)
                {
                    x = -(tag.ftcPose.bearing * (Math.PI / 180.0)) - (Math.PI / 2.0);
                    telemetry.addData("pre-x", x);
                    telemetry.addData("multiple of PI / 4", (int)((x / (Math.PI / 4.0)) + (sign(x) * 0.5)));
                    // round to nearest multiple of pi / 4 to give 8-direction in lieu of full range
                    // might?? eliminate some error? (idk)
                    x = ((int)(x / (Math.PI / 4.0) + (sign(x) * 0.5))) * (Math.PI / 4.0);
                    l = -0.5;

                    double rangeToTarget = tag.ftcPose.range - targetRange;

                    telemetry.addData("post-x", x);
                    telemetry.addData("range to target", rangeToTarget);

                    output = clamp(rangepid.update(rangeToTarget, 0), -1.0, 1.0);

                    telemetry.addData("PID OUTPUT", output);
                    telemetry.addLine(tag.ftcPose.bearing + " BEARING");
                    telemetry.addData("pid coefficients", String.format("(%s, %s, %s)", rangepid.kp, rangepid.ki, rangepid.kd));
                }
//                double l = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));

                double y1 = Math.sin(x - (Math.PI / 4.0)) * l * output;
                double y2 = Math.sin(x + (Math.PI / 4.0)) * l * output;

                telemetry.addLine(String.valueOf(tag != null));

                /*frontleft.setPower(y1);
                backright.setPower(y1);

                frontright.setPower(y2);
                backleft.setPower(y2);*/

                telemetry.update();
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
}
