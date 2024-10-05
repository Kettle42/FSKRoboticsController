package org.firstinspires.ftc.teamcode.DeepDive;

import java.util.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// import org.firstinspires.ftc.teamcode.DeepDive.XyhVector;

@TeleOp(name = "Deep Dive v1.0.0")

public class SampleOpMode extends LinearOpMode
{
    public enum OdometryPod
    {
        Left,
        Center,
        Right
    }

    private DcMotor frontright;
    private DcMotor frontleft;
    private DcMotor backright;
    private DcMotor backleft;

    private DcMotor imu;
    private DcMotor auxPod;
    private DcMotor leftPod;
    private DcMotor rightPod;

    private int oldIMU;
    private int oldRight;
    private int oldLeft;
    private int oldAux;

    private int currentIMU = 0;
    private int currentRight = 0;
    private int currentLeft = 0;
    private int currentAux = 0;

    private static double Ry = -14.3;
    private static double Ly = 14.3;
    private static double Bx = 14;
    private ArrayList<Double> vals = new ArrayList<Double>();
    double sum = 0.0;

    private static final float deadzone = 0.25f;

    private static final double c_theta = 2*Math.PI*48 / 40000;
    private static final double c_linear = 1 / 150.0;

    private final XyhVector pos = new XyhVector(0, 0, 0);

    public void runOpMode()
    {
        // init
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");

        imu = hardwareMap.get(DcMotor.class, "backright");
        //auxPod = hardwareMap.get(DcMotor.class "backright");
        //leftPod = hardwareMap.get(DcMotor.class, "backleft");
        //rightPod = hardwareMap.get(DcMotor.class, "frontright");

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);

        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // backright corresponds to the center odometry pod
        // backleft corresponds to the left odometry pod
        // frontright corresponds to the right odometry pod

        // play
        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                float power = (1 - gamepad1.right_trigger) * 0.5f;

                float lx = gamepad1.left_stick_x;
                float ly = gamepad1.left_stick_y;
                float rx = gamepad1.right_stick_x;
                if (Math.abs(lx) < deadzone) lx = 0;
                if (Math.abs(ly) < deadzone) ly = 0;
                if (Math.abs(rx) < deadzone) rx = 0;

                frontleft.setPower((-lx + ly - rx) * power);
                frontright.setPower((lx + ly + rx) * power);
                backright.setPower((-lx + ly + rx) * power);
                backleft.setPower((lx + ly - rx) * power);

                odometry();

                telemetry.addData("pos", pos.toString());
                telemetry.update();
            }
        }
    }

    public void odometry()
    {
        oldRight = currentRight;
        oldAux = currentAux;
        oldLeft = currentLeft;
        oldIMU = currentIMU;

        currentIMU = getEncoderPosition(OdometryPod.Left);

        telemetry.addLine(" " + currentIMU);

         /*currentRight = -getEncoderPosition(OdometryPod.Right);
        currentAux = getEncoderPosition(OdometryPod.Center);
        currentLeft = -getEncoderPosition(OdometryPod.Left);

        double dnR = currentRight - oldRight;
        double dnL = currentLeft - oldLeft;
        double dnB = currentAux - oldAux;

        dnR *= ((2 * Math.PI) / 2000.0) * 4.8;
        dnL *= ((2 * Math.PI) / 2000.0) * 4.8;
        dnB *= ((2 * Math.PI) / 2000.0) * 4.8;

        double dtheta = ((dnR - dnL) / 2.0) / (Ly - Ry);
        double dFwd = (dnR + dnL) / 4.0;
        double dStr = (dnB/2.0) + (Bx * dtheta);

        double dRelX = dFwd;
        double dRelY = dStr;*/

        //telemetry.addLine("" +dnB/2.0+" "+Bx*dtheta+ " " +dStr  + " " + dtheta);

        /*if (dtheta != 0) {
            double r0 = dFwd / dtheta;
            double r1 = dStr / dtheta;

            dRelX = r0 * Math.sin(dtheta) - r1 * (1 - Math.cos(dtheta));
            dRelY = r1 * Math.sin(dtheta) + r0 * (1 - Math.cos(dtheta));
        }*/

        /*pos.h += dtheta;
        pos.x += (dRelX * Math.cos(pos.h) - dRelY * Math.sin(pos.h));
        pos.y += (dRelY * Math.cos(pos.h) + dRelX * Math.sin(pos.h));*/

    }

    public int getEncoderPosition(OdometryPod pod)
    {
        if (pod == OdometryPod.Center)
            return backright.getCurrentPosition();

        if (pod == OdometryPod.Right)
            return frontright.getCurrentPosition();

        if (pod == OdometryPod.Left)
            return backleft.getCurrentPosition();

        return 0;
    }
}