package org.firstinspires.ftc.teamcode.DeepDive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KettleLibrary.ColorBrickColor;

import java.lang.Math;

@Config
@TeleOp(name = "Deep Drive v2.2.1 (new choose this)")
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
    Servo light;
    RevColorSensorV3 colorSensor;
    double deadzone = 0.25;
    Gamepad wheeler;
    Gamepad armer;
    Servo elevatorRight;
    Servo elevatorLeft;
    double wristPos = 0.0;

    Servo colorBrick;

    public static int testvar = 100;

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
        light = hardwareMap.get(Servo.class, "light");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        SampleIdentifier sampleIdentifier = new SampleIdentifier(colorSensor);

        // the wheels should stop, but not resist outside forces
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // these motors should hold their positions, resisting forces
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tricep.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorBrick = hardwareMap.get(Servo.class, "light");

        // set reversals
//        backright.setDirection(DcMotor.Direction.REVERSE);
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

        boolean canChangeElevators = false;

        HandValues.ClawMode clawMode = HandValues.ClawMode.Grabby;
        boolean canChangeClawMode = false;

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

                if (armer.y)
                {
                    if (canChangeClawMode)
                    {
                        canChangeClawMode = false;
                        clawMode = clawMode.toggle();
                    }
                }
                else
                {
                    canChangeClawMode = true;
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
                frontleft.setPower(((ly - lx) * power) + (rx * Math.abs(power)));
                frontright.setPower(((ly + lx) * power) - (rx * Math.abs(power)));
                backleft.setPower(((ly + lx) * power) + (rx * Math.abs(power)));
                backright.setPower(((ly - lx) * power) -  (rx * Math.abs(power)));


                if (colorSensor.getDistance(DistanceUnit.MM) < 60)
                {
                    SampleIdentifier.Color seenColor = sampleIdentifier.getColor();

                    double redMag = (SampleIdentifier.Color.red.subtract(seenColor)).magnitude();
                    double yellowMag = (SampleIdentifier.Color.yellow.subtract(seenColor)).magnitude();
                    double blueMag = (SampleIdentifier.Color.blue.subtract(seenColor)).magnitude();

                    double minMag = Math.min(Math.min(redMag, blueMag), yellowMag);

                    if (minMag == redMag)
                    {
                        ColorBrickColor.Red.setColorBrick(colorBrick);
                    }
                    else if (minMag == blueMag)
                    {
                        ColorBrickColor.Blue.setColorBrick(colorBrick);
                    }
                    else if (minMag == yellowMag)
                    {
                        ColorBrickColor.Yellow.setColorBrick(colorBrick);
                    }
                    else
                    {
                        ColorBrickColor.Off.setColorBrick(colorBrick);
                    }
                }
                else
                {
                    ColorBrickColor.Off.setColorBrick(colorBrick);
                }


                // the ARM!
                boolean frontArm = shoulder.getCurrentPosition() < 3160;

                if (armer.dpad_down)
                {
                    shoulder.setPower(-armPowers[j]); // rotate down
                }
                else if (armer.dpad_up)
                {
                        shoulder.setPower(armPowers[j]); // rotate up
                }
                else
                {
                    shoulder.setPower(0); // no button is pressed, stop
                }


                if (armer.dpad_right && armer.left_trigger > 0.4)
                {
                    tricep.setPower(-1); // extend
                }
                else if (armer.dpad_left)
                {
                    tricep.setPower(1); // retract
                }
                else
                {
                    tricep.setPower(0); // no button is pressed, stop
                }

                if (armer.left_bumper)
                {
                    shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    tricep.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    tricep.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }


                // setting the position for the claws and wrist
                if (!frontArm)
                {
                    wrist.setPosition(HandValues.WristMode.Folded.position);
                    hand.setPosition(HandValues.ClawMode.Clippy.position);
                }
                else
                {
//                    hand.setPosition(0.25 * (1 - armer.right_trigger));
                    hand.setPosition(clawMode.position + (clawMode.openOffset * armer.right_trigger));

                    if (armer.right_bumper && clawMode == HandValues.ClawMode.Clippy && Math.abs(getShoulderAngle()) < 30)
                    {
                        setWristAngle(-90.0);
                    }
                    else
                    {
                        setWristAngle(-getShoulderAngle());
                    }
                }


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

                telemetry.addData("Arm Angle", getShoulderAngle());
                telemetry.addData("Wrist Angle", getWristAngle());

                telemetry.addLine(wrist.getPosition() + "");
                telemetry.update();
            }
        }
    }

    private double getShoulderAngle()
    {
        return shoulder.getCurrentPosition() / 28.444;
    }

    private double getWristAngle()
    {
        return ((wrist.getPosition() - 0.64) / 0.36) * 75;
    }

    private void setWristAngle(double angle)
    {
        wrist.setPosition((0.36) * (angle / -75) + 0.64);
    }

    public static int getMotorPosition(double angle)
    {
        return (int)(28.444 * angle);
    }

    public static double getShoulderPosFromAngle(int pos)
    {
        return (28.444) * pos;
    }

    public static int getShoulderAngleFromPos(double angle)
    {
        return (int)(angle / 28.444);
    }

    public static double getWristAngleFromPos(double pos)
    {
        return ((pos - 0.64) / 0.36) * 75;
    }

    public static double getWristPosFromAngle(double angle)
    {
        return ((0.36) * (angle / -75) + 0.64);
    }
}
