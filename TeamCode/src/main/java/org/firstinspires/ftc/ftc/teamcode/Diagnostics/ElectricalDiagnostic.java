package org.firstinspires.ftc.ftc.teamcode.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ElectricalDiagnostic extends LinearOpMode {

    private DcMotor motor;
    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get("servo");
        double motorPower = 0.3;
        double servoPos = 1;

        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(motorPower);
            servo.setPosition(servoPos);

            motorPower *= -1;
            servoPos *= -1;

        }
    }

    public void telemetry() {

    }
}