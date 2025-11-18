package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PIDs.Outtake;

@TeleOp(name="TeleOp Shooter", group="TeleOp")
public class TeleOpShooter extends OpMode {
    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;
    private DcMotor intake, backIntake;
    private ElapsedTime time;
    private DcMotorEx outtake, outtake2;
    private double tolerance;
    double previousTime;
    double currentTime;
    double previousError;
    double error;
    double Kp;
    double Ki;
    double Kd;
    double max_i;
    double min_i;
    double motorVelocity;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        intake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        backIntake.setDirection(DcMotor.Direction.REVERSE);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);

        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        tolerance = 0.05;
        previousTime = 0;
        previousError = 0;
        Kp = 0.1;
        Ki = 0;
        Kd = 0;
        max_i = 0.03;
        min_i = -0.03;
        motorVelocity = 0;
        currentTime = 0;
        error = 0;
        time = new ElapsedTime();
    }

    @Override
    public void loop() {
        float drive = gamepad1.left_stick_y;
        float turn = -gamepad1.right_stick_x;
        float strafe = gamepad1.left_stick_x;

        double flPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double frPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double blPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double brPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        if (gamepad1.right_trigger > 0.0) {
            flPower *= 0.5;
            frPower *= 0.5;
            blPower *= 0.5;
            brPower *= 0.5;
        }

        if (gamepad1.right_bumper) {
            outtake2.setVelocity(1600);
            telemetry.addData("Velocity: ", outtake2.getVelocity());
            telemetry.update();
        }

        if (gamepad1.left_bumper) {
            flPower *= 0.25;
            frPower *= 0.25;
            blPower *= 0.25;
            brPower *= 0.25;
        }

        if (gamepad1.left_bumper) {
            intake.setPower(1);
            backIntake.setPower(1);
        }

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void updatePID(double targetPower) {
        double p = 0;
        double i = 0;
        double d = 0;
        double motorPower;

        double currentPower = (outtake.getPower() + outtake2.getPower()) / 2;

        while (Math.abs(targetPower - currentPower) > tolerance) {
            currentTime = time.milliseconds();
            error = targetPower - currentPower;
            if (error > tolerance) {
                p = Kp * error;
                i += (Ki * (error * (currentTime - previousTime)));
                i = Range.clip(i, min_i, max_i);
                d = Kd * (error - previousError) / (currentTime - previousTime);
                motorPower = p + i + d + targetPower;
            } else {
                motorPower = 0;
            }

            previousError = error;
            previousTime = currentTime;
            currentPower = (double)outtake.getVelocity();
            outtake.setPower(motorPower);
            outtake2.setPower(motorPower);
            telemetry.addData("Outtake Power: ", outtake.getPower());
            telemetry.addData("Outtake2 Power: ", outtake2.getPower());
            telemetry.update();
        }
    }
}
