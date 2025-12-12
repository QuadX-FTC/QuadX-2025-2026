package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;


import org.firstinspires.ftc.teamcode.PIDs.Drivetrain;

public class SpecializedAutoFunctions {

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
    double Kp = -3;
    double Ki = -10;
    double Kd = -0.5;
    double max_i;
    double min_i;
    double motorPower;
    double currentVelocity;
    double outtakeVelocity;
    double specialKP = 30;
    double specialKI = 15;
    double specialKD = 25;
    double specialtolerance = 10;
    private IMU imu;
    private ElapsedTime time;
    private Limelight3A Lemon;

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

    public void ShooterPIDSpinUp(double targetVelocity, DcMotorEx outtake, DcMotorEx outtake2, ElapsedTime time1) {
        double p = 0;
        double i = 0;
        double d = 0;


        currentVelocity = outtake.getVelocity();
        //if (Math.abs(targetVelocity - currentVelocity) > specialtolerance) {
        if ((Math.abs(targetVelocity - currentVelocity) > specialtolerance)) {
            currentTime = time1.milliseconds();
            error = targetVelocity - currentVelocity;
            if (error > 0) {
                p = specialKP * error;
                i += (specialKI * (error * (currentTime - previousTime)));
                i = Range.clip(i, min_i, max_i);
                d = specialKD * (error - previousError) / (currentTime - previousTime);
                outtakeVelocity = p + i + d;
            }

            previousError = error;
            previousTime = currentTime;
            currentVelocity = outtake.getVelocity();
            outtake.setVelocity(outtakeVelocity);
            outtake2.setVelocity(outtakeVelocity);
        }
    }

    public int ObeliskScan(int IntPipeLineIndex, Limelight3A limelight, int choicePipeline) {

        limelight.pipelineSwitch(choicePipeline);
        LLResult results = limelight.getLatestResult();
        int id = 0;
        if (results.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = (List<LLResultTypes.FiducialResult>) results.getFiducialResults();
            id = 0;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                id = fiducial.getFiducialId();
            }

            return id;
        }

        limelight.pipelineSwitch(IntPipeLineIndex);
        return 0;
    }
}

