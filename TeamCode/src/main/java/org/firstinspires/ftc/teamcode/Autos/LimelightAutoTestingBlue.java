package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PIDs.Drivetrain;

@Autonomous(name = "Testing lmao", group = "Autonomous")
public class LimelightAutoTestingBlue extends LinearOpMode {
    private DcMotor fr, fl, br, bl;
    private boolean straightAssist = false;
    private boolean lastToggle = false;
    private HuskyLens Husky;
    private Drivetrain drivetrainMacros;
    double tx;
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
    double motorPower;
    private IMU imu;
    private ElapsedTime time;
    private Limelight3A Lemon;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();


        int[] validIDs = {20,24};
        Lemon = hardwareMap.get(Limelight3A.class,"limelight");
        Lemon.setPollRateHz(100); //this is how many times we ask the Limelight for information PER SECOND.
        Lemon.start();
        Lemon.pipelineSwitch(0);

        LimeAlignBlue();
    }

    public void LimeAlignBlue() {

        double targetHeading = -4;
        double p = 0;
        double i = 0;
        double d = 0;

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double heading = Lemon.getLatestResult().getTx();

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
            heading = Lemon.getLatestResult().getTx();
            fl.setPower(motorPower);
            fr.setPower(motorPower);
            bl.setPower(motorPower * -1);
            fr.setPower(motorPower * -1);
        }

    }

}
