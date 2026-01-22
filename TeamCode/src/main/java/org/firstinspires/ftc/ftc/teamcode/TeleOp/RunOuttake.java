package org.firstinspires.ftc.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="RunOuttake+Intake", group="TeleOp")
public class RunOuttake extends OpMode {
    private DcMotorEx outtake, outtake2, frontIntake, backIntake;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");
    }

    @Override
    public void loop() {
        outtake.setPower(.58);
        outtake2.setPower(.58);
        frontIntake.setPower(1);
        backIntake.setPower(1);
    }
}