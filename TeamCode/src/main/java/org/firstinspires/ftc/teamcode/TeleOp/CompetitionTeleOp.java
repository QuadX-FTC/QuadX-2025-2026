package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="CompetitionTeleOp", group="TeleOp")
public class CompetitionTeleOp extends OpMode{
    private DcMotorEx outtake, outtake2, fl, fr, bl, br, frontIntake, backIntake;
    private Servo shroud;

    // control state variables for toggles
    boolean gamepad1DpadUpWasPressed = false;
    boolean gamepad1DpadDownWasPressed = false;
    boolean gamepad2DpadUpWasPressed = false;
    boolean gamepad2DpadDownWasPressed = false;

    // init states for outtake and shroud
    double targetOuttakePwr = 0.8;
    double pos = 0.5;
    // outtake toggle
    boolean outtakeOn = false;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shroud = hardwareMap.get(Servo.class, "shroudCont");
        shroud.setPosition(pos); // Set initial position

        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Outtake power toggle
        // dpad up to increase by 5%, dpad down to decrease by 5%
        if (gamepad1.dpad_up && !gamepad1DpadUpWasPressed) {
            targetOuttakePwr += 0.05; // Increment by 5%
            targetOuttakePwr = Range.clip(targetOuttakePwr, 0.0, 1.0);
            telemetry.addData("Outtake Target Power", targetOuttakePwr);
            telemetry.update();
        }
        gamepad1DpadUpWasPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !gamepad1DpadDownWasPressed) {
            targetOuttakePwr -= 0.05; // Decrement by 5%
            targetOuttakePwr = Range.clip(targetOuttakePwr, 0.0, 1.0);
            telemetry.addData("Outtake Target Power", targetOuttakePwr);
            telemetry.update();
        }
        gamepad1DpadDownWasPressed = gamepad1.dpad_down;

        //Outtake
        //right trigger to turn on shooter, left trigger to turn on shooter (no need to hold down trigger)
        if(gamepad2.right_trigger > 0.05){
            outtakeOn = true;
            telemetry.addData("shooter state", "ON");
        }
        if(gamepad2.left_trigger > 0.05){
            outtakeOn = false;
            telemetry.addData("shooter state", "OFF");
        }
        if(outtakeOn){
            outtake.setPower(targetOuttakePwr);
            outtake2.setPower(targetOuttakePwr);
        }
        if(!outtakeOn) {
            outtake.setPower(0);
            outtake2.setPower(0);
        }

        //Drive
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        double flPower = drive + turn - strafe;
        double frPower = drive - turn + strafe;
        double blPower = drive + turn + strafe;
        double brPower = drive - turn - strafe;

        fl.setPower(Range.clip(flPower, -1.0, 1.0));
        fr.setPower(Range.clip(frPower, -1.0, 1.0));
        bl.setPower(Range.clip(blPower, -1.0, 1.0));
        br.setPower(Range.clip(brPower, -1.0, 1.0));

        //Intake
        if (gamepad2.right_bumper) {
            frontIntake.setPower(1);
            backIntake.setPower(1);
        } else if (gamepad2.left_bumper) {
            frontIntake.setPower(-1);
            backIntake.setPower(-1);
        } else if (gamepad2.a) {
            frontIntake.setPower(1);
        }else if (gamepad2.b) {
            backIntake.setPower(1);
        } else if (gamepad2.x) {
            frontIntake.setPower(-1);
        }else if (gamepad2.y) {
            backIntake.setPower(-1);
        } else {
            frontIntake.setPower(0);
            backIntake.setPower(0);
        }

        // Shroud
        if (gamepad2.dpad_up && !gamepad2DpadUpWasPressed) {
            pos += 0.1;
            pos = Range.clip(pos, 0.0, 1.0);
            shroud.setPosition(pos);
            telemetry.addData("shroud pos", pos);
        }
        gamepad2DpadUpWasPressed = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !gamepad2DpadDownWasPressed) {
            pos -= 0.1;
            pos = Range.clip(pos, 0.0, 1.0);
            shroud.setPosition(pos);
            telemetry.addData("shroud pos", pos);
        }
        gamepad2DpadDownWasPressed= gamepad2.dpad_down;

        telemetry.update();
    }
}