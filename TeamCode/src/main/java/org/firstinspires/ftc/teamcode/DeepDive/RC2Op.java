package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RC 2 Initial test")
public class RC2Op extends LinearOpMode
{
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx frontleft;
    private DcMotorEx frontright;
    private DcMotorEx shoulder;
    private DcMotorEx tricep;
    private DcMotorEx[] wheels;

    @Override
    public void runOpMode()
    {
        // put your initialization blocks here
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        tricep = hardwareMap.get(DcMotorEx.class, "tricep");

        wheels = new DcMotorEx[] { backleft, backright, frontleft, frontright };

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                // put your loop blocks here
                double lx = gamepad1.left_stick_x;
                double ly = gamepad1.left_stick_y;
                double rx = gamepad1.right_stick_x;
                double power = 1;

                backleft.setPower((ly - rx + lx) * power);
                backright.setPower((ly +  rx - lx) * power);
                frontleft.setPower((ly - rx - lx) * power);
                frontright.setPower((ly + rx + lx) * power);

                if (gamepad1.dpad_up)
                {
                    shoulder.setPower(-1);
                }
                else if (gamepad1.dpad_down)
                {
                    shoulder.setPower(1);
                }
                else
                {
                    shoulder.setPower(0);
                }

                if (gamepad1.dpad_right)
                {
                    tricep.setPower(-1);
                }
                else if (gamepad1.dpad_left)
                {
                    tricep.setPower(1);
                }
                else
                {
                    tricep.setPower(0);
                }

                for (DcMotorEx wheel : wheels)
                {
                    telemetry.addData(wheel.getDeviceName(), wheel.getPower());
                }

                if (gamepad1.right_trigger > 0.2)
                {
                    backleft.setPower(1);
                }

                telemetry.update();
            }
        }
    }
}
