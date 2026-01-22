package org.firstinspires.ftc.ftc.teamcode.PIDs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain {
    private ElapsedTime time;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private IMU imu;
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

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);


        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void turn(double targetHeading) {
        init();
        double p = 0;
        double i = 0;
        double d = 0;

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double heading = robotOrientation.getYaw(AngleUnit.DEGREES);

        while (Math.abs(targetHeading - heading) > tolerance) {
            currentTime = time.milliseconds();
            error = targetHeading - heading;
            p = Kp * error;
            i += (Ki * (error * (currentTime - previousTime)));
            i = Range.clip(i, min_i, max_i);
            d = Kd * (error - previousError) / (currentTime - previousTime);

            motorPower = p + i + d;

            previousError = error;
            previousTime = currentTime;
            heading = robotOrientation.getYaw(AngleUnit.DEGREES);
            fl.setPower(motorPower);
            fr.setPower(motorPower * -1);
            bl.setPower(motorPower);
            fr.setPower(motorPower * -1);
        }
    }
}