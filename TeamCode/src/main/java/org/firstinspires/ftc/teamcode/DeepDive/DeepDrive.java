package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

@TeleOp(name = "Deep Drive v1.1.0 (prod)")
public class DeepDrive extends LinearOpMode
{
    DcMotorEx frontleft;
    DcMotorEx frontright;
    DcMotorEx backleft;
    DcMotorEx backright;
    DcMotorEx[] wheels;
    DcMotorEx shoulder;
    DcMotor tricep;
    Servo hand;
    Servo wrist;
    double deadzone = 0.25;
    Gamepad wheeler;
    Gamepad armer;
    Servo elevatorRight;
    Servo elevatorLeft;

//    WebcamName webcam; // this will be used when a camera is (eventually) attached to the robot
//

    public void runOpMode()
    {
        // get motors from hardware configuration
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        hand = hardwareMap.get(Servo.class, "hand");
        wrist = hardwareMap.get(Servo.class, "wrist");
        tricep = hardwareMap.get(DcMotor.class, "tricep");

        elevatorRight = hardwareMap.get(Servo.class, "elevatorRight");
        elevatorLeft = hardwareMap.get(Servo.class, "elevatorLeft");

        // the wheels should stop, but not resist outside forces
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // these motors should hold their positions, resisting forces
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tricep.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set reversals
        backright.setDirection(DcMotor.Direction.REVERSE);
//        backleft.setDirection(DcMotor.Direction.REVERSE);
//        frontright.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);

        // array of wheels so that operations that should apply to all wheels can be done easier
        wheels = new DcMotorEx[] {frontleft, frontright, backleft, backright};
        for (DcMotorEx wheel : wheels)
        {
            wheel.setVelocityPIDFCoefficients(1.0, 1.0, 1.0, 0.0);
        }

//        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        boolean reversed = true; // whether or not the front or back is considered "forward"
        boolean canReverse = true;

        double[] powers = new double[]{0.1, 0.25, 0.5, 0.66}; // speed coefficients
        int i = powers.length - 1; // coefficient index
        boolean canChangePower = true;

        double[] armPowers = new double[] {0.25, 0.5, 1};
        int j = armPowers.length - 1;
        boolean canChangeArmPower = true;

        boolean elevators = false;
        boolean canChangeElevators = false;

        wheeler = gamepad1;
        armer = gamepad2;

        ElapsedTime time = new ElapsedTime();

        telemetry.addLine(wrist.getPosition() + "");
        telemetry.update();

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive()) {
                // handle button events
                if (wheeler.y)
                {
                    if (canReverse)
                    {
                        reversed = !reversed;
                        canReverse = false;
                        wheeler.rumble(200); // alert to reversal
                    }
                }
                else
                {
                    canReverse = true; // reset ability to reverse
                }

                // change the robot speed coefficient index
                if (wheeler.x)
                {
                    if (canChangePower)
                    {
                        i = Math.max(0, i - 1);
                        canChangePower = false; // so the robot does not change it every tick
                    }
                }
                else if (wheeler.b)
                {
                    if (canChangePower)
                    {
                        canChangePower = false;
                        i = Math.min(powers.length - 1, i + 1);
                    }
                }
                else
                {
                    canChangePower = true; // reset ability to change index
                }

                if (time.seconds() > 80 && time.seconds() < 90)
                {
                    if ((((int)time.seconds()) % 2) == 0)
                    {
                        wheeler.rumble(500);
                        armer.rumble(500);
                    }
                }

                if (time.seconds() < 90 && !wheeler.left_bumper) canChangeElevators = false;
                if (wheeler.dpad_down)
                {
                    if (canChangeElevators)
                    {
                        elevatorRight.setPosition(0);
                        elevatorLeft.setPosition(0);
                        canChangeElevators = false;
                    }
                }
                else if (wheeler.dpad_up)
                {
                    if (canChangeElevators)
                    {
                        elevatorRight.setPosition(1);
                        elevatorLeft.setPosition(1);
                        canChangeElevators = false;
                    }
                }
                else
                {
                    canChangeElevators = true;
                }

                if (armer.x)
                {
                    if (canChangeArmPower)
                    {
                        j = Math.max(0, j - 1);
                        canChangeArmPower = false;
                    }
                }
                else if (armer.b)
                {
                    if (canChangeArmPower)
                    {
                        j = Math.min(armPowers.length - 1, j + 1);
                        canChangeArmPower = false;
                    }
                }
                else
                {
                    canChangeArmPower = true;
                }

                double bright = (i + 1f) / powers.length;
                if (reversed) wheeler.setLedColor(0.5 * bright, 1 * bright, 0 * bright, -1);
                else wheeler.setLedColor(1, 0.5 * bright, 0, -1);

                bright = (j + 1f) / armPowers.length;
                armer.setLedColor(0.5 * bright, 0, 1 * bright, -1);

                // drive the robot
                double power = powers[i]; // get power coefficient
                if (reversed) power *= -1; // reverse if reversed
                double lx = wheeler.left_stick_x;
                double ly = wheeler.left_stick_y;
                double rx = wheeler.right_stick_x;
                // adjust for deadzone
                if (Math.abs(lx) < deadzone) lx = 0;
                if (Math.abs(ly) < deadzone) ly = 0;
                if (Math.abs(rx) < deadzone) rx = 0;

                // set wheel powers, adjusted for reversable front
                frontleft.setPower(((ly + lx) * power) - (rx * Math.abs(power)));
                frontright.setPower(((ly - lx) * power) + (rx * Math.abs(power)));
                backleft.setPower(((ly + lx) * power) + (rx * Math.abs(power)));
                backright.setPower(((ly - lx) * power) -  (rx * Math.abs(power)));

                // the ARM!
                boolean frontArm = shoulder.getCurrentPosition() > -3558;
                int maxExt = 0;
                if (frontArm)
                {
                    maxExt = -600;
                }
                if (Math.abs(shoulder.getCurrentPosition() + 3558) < 300)
                {
                    maxExt = -3638;
                }

                if (armer.dpad_down)
                {
                    shoulder.setPower(armPowers[j]); // go down
                }
                else if (armer.dpad_up)
                {
                    if (tricep.getCurrentPosition() > maxExt)
                    {
                        shoulder.setPower(-armPowers[j]); // go up
                    }
                    else
                    {
                        armer.rumble(100);
                        shoulder.setPower(0);
                    }
                }
                else
                {
                    shoulder.setPower(0); // no button is pressed, stop
                }

                // the tricep
                if (armer.dpad_left)
                {
                    tricep.setPower(1); // retract
                }
                else if (armer.dpad_right)
                {
                    if (tricep.getCurrentPosition() > maxExt)
                    {
                        tricep.setPower(-1); // extend
                    }
                    else
                    {
                        armer.rumble(100);
                        tricep.setPower(0);
                    }
                }
                else
                {
                    tricep.setPower(0); // no button is pressed, stop
                }

                telemetry.addData("can extend", tricep.getCurrentPosition() > maxExt);

                if (armer.y)
                {
                    shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    tricep.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    tricep.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                // setting the position for the claws and wrist
                hand.setPosition(0.25 * (1 - armer.right_trigger));
                wrist.setPosition(0.66 * armer.left_trigger);

                // button on the gamepad to stop the robot
                if (wheeler.touchpad || armer.touchpad)
                {
                    requestOpModeStop();
                }

                telemetry.addData("Reversed", reversed);
                telemetry.addData("i", i);
                telemetry.addData("Speed", power);

                telemetry.addData("j", j);
                telemetry.addData("Arm Speed", armPowers[j]);

                telemetry.addData("Arm Pos", shoulder.getCurrentPosition());
                telemetry.addData("Tricep Pos", tricep.getCurrentPosition());

                telemetry.addLine(wrist.getPosition() + "");
                telemetry.update();
            }
        }
    }
}