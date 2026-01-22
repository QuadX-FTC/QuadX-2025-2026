package org.firstinspires.ftc.ftc.teamcode.PIDs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Lift {
    private ElapsedTime time;
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private int tolerance;
    double previousTime;
    double currentTime;
    double previousError;
    double error;
    double Kp;
    double Ki;
    double Kd;
    double max_i;
    double min_i;
    double motorPower;

    public void init() {
        time = new ElapsedTime();

        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("rightLeft");

        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tolerance = 3;
        previousTime = 0;
        previousError = 0;
        Kp = 0;
        Ki = 0;
        Kd = 0;
        max_i = 0.2;
        min_i = -0.2;
        motorPower = 0;
        currentTime = 0;
        error = 0;
    }

    public void lift(double targetPos) {
        init();
        double p = 0;
        double i = 0;
        double d = 0;

        double currentPos = (double)(liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2;

        while (Math.abs(targetPos - currentPos) > tolerance) {
            currentTime = time.milliseconds();
            error = targetPos - currentPos;
            p = Kp * error;
            i += (Ki * (error * (currentTime - previousTime)));
            i = Range.clip(i, min_i, max_i);
            d = Kd * (error - previousError) / (currentTime - previousTime);

            motorPower = p + i + d;

            previousError = error;
            previousTime = currentTime;
            currentPos = (double)(liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2;
            liftLeft.setPower(motorPower);
            liftRight.setPower(motorPower);
        }
    }
}