package org.firstinspires.ftc.ftc.teamcode.PIDs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Turn {
    private ElapsedTime time;
    private BNO055IMU imu;
    private Orientation angles;
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

    public void turn(double targetHeading, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        init();
        double p = 0;
        double i = 0;
        double d = 0;

        while (Math.abs(targetHeading - angles.firstAngle) > tolerance) {
            currentTime = time.milliseconds();
            error = targetHeading - angles.firstAngle;
            p = Kp * error;
            i += (Ki * (error * (currentTime - previousTime)));
            i = Range.clip(i, min_i, max_i);
            d = Kd * (error - previousError) / (currentTime - previousTime);

            motorPower = p + i + d;

            previousError = error;
            previousTime = currentTime;
            angles = imu.getAngularOrientation();
            fl.setPower(motorPower);
            fr.setPower(-motorPower);
            bl.setPower(motorPower);
            br.setPower(-motorPower);
        }
    }
}