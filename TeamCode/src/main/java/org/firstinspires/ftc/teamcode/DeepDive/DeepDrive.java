package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.lang.Math;

@TeleOp(name = "Deep Drive")
public class DeepDrive extends LinearOpMode
{
    DcMotorEx frontleft;
    DcMotorEx frontright;
    DcMotorEx backleft;
    DcMotorEx backright;
    DcMotorEx[] wheels;
    DcMotor leftshoulder;
    DcMotor rightshoulder;
    DcMotor tricep;
    Servo hand;
    Servo wrist;
    float deadzone = 0.25f;

    WebcamName webcam;

    public void runOpMode() {
        // get motors from hardware configuration
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        rightshoulder = hardwareMap.get(DcMotor.class, "rightshoulder");
        leftshoulder = hardwareMap.get(DcMotor.class, "leftshoulder");
        hand = hardwareMap.get(Servo.class, "hand");
        wrist = hardwareMap.get(Servo.class, "wrist");
        tricep = hardwareMap.get(DcMotor.class, "tricep");

        // set braking
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightshoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftshoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tricep.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set reversals
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        rightshoulder.setDirection(DcMotor.Direction.REVERSE);

        wheels = new DcMotorEx[] {frontleft, frontright, backleft, backright};
        for (DcMotorEx wheel : wheels) {
            wheel.setVelocityPIDFCoefficients(1.0, 1.0, 1.0, 0.0);
        }

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // drive the robot
                float power = 0.5f;
                float lx = gamepad1.left_stick_x;
                float ly = gamepad1.left_stick_y;
                float rx = gamepad1.right_stick_x;
                if (Math.abs(lx) < deadzone) lx = 0;
                if (Math.abs(ly) < deadzone) ly = 0;
                if (Math.abs(rx) < deadzone) rx = 0;

                frontleft.setPower((ly - lx - rx) * power);
                frontright.setPower((ly + lx + rx) * power);
                backleft.setPower((ly + lx - rx) * power);
                backright.setPower((ly - lx + rx) * power);

                /*frontleft.setVelocity(100.0);
                frontright.setVelocity(-100.0);
                backleft.setVelocity(100.0);
                backright.setVelocity(-100.0);*/

                // the ARM!
                if (gamepad1.dpad_down) {
                    setArmPower(0.5f);
                }
                else if (gamepad1.dpad_up)
                {
                    setArmPower(-0.5f);
                }
                else
                {
                    setArmPower(0);
                }

                // the tricep
                if (gamepad1.dpad_left)
                {
                    tricep.setPower(0.5);
                }
                else if (gamepad1.dpad_right)
                {
                    tricep.setPower(-0.5);
                }
                else
                {
                    tricep.setPower(0);
                }

                hand.setPosition(gamepad1.right_trigger);
                wrist.setPosition(gamepad1.left_trigger);
            }
        }
    }

    public void setArmPower(float armPower) {
        rightshoulder.setPower(armPower);
        leftshoulder.setPower(armPower);
    }
}