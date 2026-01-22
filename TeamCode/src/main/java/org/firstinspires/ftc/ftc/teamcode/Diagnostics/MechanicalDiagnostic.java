package org.firstinspires.ftc.ftc.teamcode.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class MechanicalDiagnostic extends LinearOpMode {

    private DcMotor motor;
    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get("servo");

        double motorPower = 0.5;
        double servoPosition = 0.2;

        waitForStart();

        while(opModeIsActive()) {
            motor.setPower(motorPower);
            servo.setPosition(servoPosition);
            telemetry(motor.getPower(), servo.getPosition(), motorPower, servoPosition);
            motorPower *= -1;
            servoPosition *= -1;
        }
    }

    public void telemetry(double currentPower, double currentPos, double targetPower, double targetPos) {
        telemetry.addData("Motor Power: ", currentPower);
        telemetry.addData("Motor Power: ", targetPower);
        telemetry.addData("Servo Position: ", currentPos);
        telemetry.addData("Servo Position: ", targetPos);
    }
}