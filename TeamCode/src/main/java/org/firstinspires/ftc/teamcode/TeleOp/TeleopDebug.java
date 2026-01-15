package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="TeleopDebug", group="TeleOp")
public class TeleopDebug extends OpMode {

    private DcMotorEx fl, fr, bl, br;

    @Override
    public void init() {

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "br");
        br = hardwareMap.get(DcMotorEx.class, "bl");

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {


        float drive = -gamepad1.left_stick_y;
        float strafe = gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;

        double frPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double flPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double brPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double blPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        if (gamepad1.b) {
            fl.setPower(1);
        } else {
            fl.setPower(0);
        }
        if (gamepad1.a) {
            bl.setPower(1);
        } else {
            bl.setPower(0);
        }
        if (gamepad1.x) {
            fr.setPower(1);
        } else {
            fr.setPower(0);
        }
        if (gamepad1.y) {
            br.setPower(1);
        } else {
            br.setPower(0);
        }

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);


    }
}