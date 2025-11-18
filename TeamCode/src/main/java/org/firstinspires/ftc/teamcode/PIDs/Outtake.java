package org.firstinspires.ftc.teamcode.PIDs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Outtake {
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

    public void init() {
        time = new ElapsedTime();

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        tolerance = 0.05;
        previousTime = 0;
        previousError = 0;
        Kp = 0;
        Ki = 0;
        Kd = 0;
        max_i = 0.03;
        min_i = -0.03;
        motorVelocity = 0;
        currentTime = 0;
        error = 0;
    }

    public void outtake(double targetVelocity, double Kp, double Ki, double Kd) {
        init();
        double p = 0;
        double i = 0;
        double d = 0;

        double currentVelocity = (double)outtake.getVelocity();

        while (Math.abs(targetVelocity - currentVelocity) > tolerance) {
            currentTime = time.milliseconds();
            error = targetVelocity - currentVelocity;
            if (error > 0) {
                p = Kp * error;
                i += (Ki * (error * (currentTime - previousTime)));
                i = Range.clip(i, min_i, max_i);
                d = Kd * (error - previousError) / (currentTime - previousTime);
                motorVelocity = p + i + d;
            } else {
                motorVelocity = 0;
            }

            previousError = error;
            previousTime = currentTime;
            currentVelocity = (double)outtake.getVelocity();
            outtake.setVelocity(motorVelocity);
            outtake2.setVelocity(motorVelocity);
            telemetry.addData("Outtake Velocity: ", outtake.getVelocity());
            telemetry.addData("Outtake2 Velocity: ", outtake2.getVelocity());
            telemetry.update();
        }
    }
}