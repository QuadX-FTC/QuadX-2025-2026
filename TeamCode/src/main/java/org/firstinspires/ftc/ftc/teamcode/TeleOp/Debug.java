package org.firstinspires.ftc.ftc.teamcode.TeleOp;

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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Debug", group="TeleOp")
public class Debug extends OpMode {
    // Hardware
    private DcMotorEx outtake, outtake2, fl, fr, bl, br, frontIntake, backIntake;
    private Servo shroud, turret1, turret2;
    private Limelight3A lemon;
    private IMU imu;

    // State Variables
    private boolean outtakeOn = false;
    private double targetOuttakePwr = 0.8;
    private double shroudPos = 0.5;
    private double turretPos = 0;

    // Toggle Helpers (Rising Edge Detectors)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // PID Shooter Variables
    private boolean shooterPIDEnabled = false;
    private boolean lastLeftTrigger = false;
    private double targetVelocity = 2000; // Target velocity in ticks/sec (adjust as needed)
    private double specialKP = 0.001; // Tune these based on your shooter
    private double specialKI = 0.0001;
    private double specialKD = 0.01;
    private double specialtolerance = 50;
    private double min_i = 0;
    private double max_i = 100;
    private double currentVelocity = 0;
    private double error = 0;
    private double previousError = 0;
    private double previousTime = 0;
    private double iAccumulator = 0;
    private double outtakeVelocity = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Drive Motors
        fl = setupMotor("fl", DcMotorSimple.Direction.REVERSE);
        bl = setupMotor("bl", DcMotorSimple.Direction.REVERSE); //br, REVERSE (V2)
        fr = setupMotor("fr", DcMotorSimple.Direction.FORWARD);
        br = setupMotor("br", DcMotorSimple.Direction.FORWARD); //bl, FORWARD (V2)

        // Outtake
        outtake = setupMotor("outtake", DcMotorSimple.Direction.REVERSE);
        outtake2 = setupMotor("outtake2", DcMotorSimple.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake
        frontIntake = setupMotor("frontIntake", DcMotorSimple.Direction.REVERSE);
        backIntake = setupMotor("backIntake", DcMotorSimple.Direction.REVERSE);

        /*
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        */
        //shroud = hardwareMap.get(Servo.class, "shroudCont");
        //shroud.setPosition(shroudPos);

        // Sensors
        lemon = hardwareMap.get(Limelight3A.class, "limelight");
        lemon.pipelineSwitch(0);
        lemon.setPollRateHz(100);
        lemon.start();

        initIMU();

        previousTime = getRuntime();
    }

    @Override
    public void loop() {
        handleOuttakeControls();
        handleIntakeControls();
        handleDrive();
        handleLimelight();
        handleAprilTagAlign();

        telemetry.update();
    }

    private void handleOuttakeControls() {
        if (gamepad2.a && !lastLeftTrigger) {
            shooterPIDEnabled = !shooterPIDEnabled;
            iAccumulator = 0; // Reset integral accumulator on toggle
        }
        lastLeftTrigger = gamepad2.back;

        if (shooterPIDEnabled) {
            // PID Control Mode
            double currentTime = getRuntime();
            currentVelocity = outtake.getVelocity();
            error = targetVelocity - currentVelocity;

            if (Math.abs(error) > specialtolerance) {
                double dt = currentTime - previousTime;

                // Calculate PID components
                double p = specialKP * error;
                iAccumulator += specialKI * error * dt;
                iAccumulator = Range.clip(iAccumulator, min_i, max_i);
                double d = specialKD * (error - previousError) / dt;

                outtakeVelocity = p + iAccumulator + d;
            } else {
                outtakeVelocity = targetVelocity; // Hold target when in tolerance
            }

            outtakeVelocity = Range.clip(outtakeVelocity, 0, 3000);

            previousError = error;
            previousTime = currentTime;

            telemetry.addData("Shooter Mode", "PID");
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Error", error);

            outtake.setVelocity(outtakeVelocity);
            outtake2.setVelocity(outtakeVelocity);
        } else {
            // Manual Power Mode
            // Increment/Decrement Power with proper toggle logic
            if (gamepad1.dpad_up && !lastDpadUp) targetOuttakePwr = Range.clip(targetOuttakePwr + 0.05, 0, 1);
            if (gamepad1.dpad_down && !lastDpadDown) targetOuttakePwr = Range.clip(targetOuttakePwr - 0.05, 0, 1);

            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // Trigger Toggles
            if (gamepad2.right_trigger > 0.1) outtakeOn = true;
            else if (gamepad2.left_trigger > 0.1) outtakeOn = false;

            outtakeVelocity = outtakeOn ? targetOuttakePwr : 0;

            telemetry.addData("Shooter Mode", "Manual");
            telemetry.addData("Shooter", outtakeOn ? "ON" : "OFF");
            telemetry.addData("Outtake Power", targetOuttakePwr);
            outtake.setPower(outtakeVelocity);
            outtake2.setVelocity(outtakeVelocity);
        }
    }

    private void handleIntakeControls(){
        if (gamepad2.left_bumper){
            frontIntake.setPower(1);
            backIntake.setPower(-0.3);
        }else if (gamepad2.right_bumper){
            frontIntake.setPower(1);
            backIntake.setPower(1);
        }else{
            frontIntake.setPower(0);
            backIntake.setPower(0);
        }
        /*
        if(gamepad2.b){
            turret1.setPosition(turretPos-=0.005);
            turret2.setPosition(turretPos-=0.005);
        }else if(gamepad2.x){
            turret1.setPosition(turretPos+=0.005);
            turret2.setPosition(turretPos+=0.005);
        }*/
    }

    private void handleDrive() {
        if(gamepad1.a){
            fl.setPower(1); //
        } else if(gamepad1.b){
            fr.setPower(1);
        } else if(gamepad1.x){
            bl.setPower(1);
        } else if(gamepad1.y){
            br.setPower(1);
        } else{
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
    }

    private void handleLimelight() {
        LLResult result = lemon.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("Target X", result.getTx());
            telemetry.addData("Botpose", result.getBotpose().toString());
        } else {
            telemetry.addData("Limelight", "No Target");
        }
    }

    private DcMotorEx setupMotor(String name, DcMotorSimple.Direction dir) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(dir);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void initIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    // Add these constants/variables to your class
    private final double ALIGN_KP = 0.04; // Proportional gain: adjust if it oscillates or is too slow
    private final double ALIGN_TOLERANCE = 1.0; // Degrees of error allowed

    /**
     * Calculates the rotation power needed to center the AprilTag.
     * Returns a value between -1.0 and 1.0.
     */
    private double handleAprilTagAlign() {
        LLResult result = lemon.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // Horizontal offset in degrees

            if (Math.abs(tx) > ALIGN_TOLERANCE) {
                // Power is proportional to the error (tx)
                double rotationOutput = tx * ALIGN_KP;

                // Limit output to prevent aggressive spinning
                return Range.clip(rotationOutput, -0.5, 0.5);
            }
        }
        return 0; // No target or already aligned
    }

}