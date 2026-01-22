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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="V2-CompetitionTeleOpBlue", group="TeleOp")
public class CompetitionTeleOpBlue extends OpMode {
    // Hardware
    private DcMotorEx outtake, outtake2, fl, fr, bl, br, frontIntake, backIntake;
    private Servo shroud, turret1, turret2;
    private Limelight3A lemon;
    private IMU imu;
    private ElapsedTime time;
    // State Variables
    private boolean outtakeOn = false;
    private boolean gamepad2DpadUpWasPressed = false;
    private boolean gamepad2DpadDownWasPressed = false;
    private double targetOuttakePwr = 0.8;
    private double shroudPos = 0.5;
    private double turretPos = 0;

    // Toggle Helpers (Rising Edge Detectors)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // PID Shooter Variables
    private boolean shooterPIDEnabled = false;
    private boolean lastLeftTrigger = false;
    private double targetVelocity = 1550; // Target velocity in ticks/sec (adjust as needed)
    private double specialKP = 24; // Tune these based on your shooter
    private double specialKI = 100;
    private double specialKD = 110;
    private double specialtolerance = 10;
    private double p = 0;
    private double d = 0;
    private double i = 0;
    private double currentVelocity = 0;
    private double error = 0;
    private double previousError = 0;
    private double previousTime = 0;
    private double iAccumulator = 0;
    private double outtakeVelocity = 0;
    private double currentTime;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Drive Motors
        fl = setupMotor("fl", DcMotorSimple.Direction.REVERSE);
        bl = setupMotor("bl", DcMotorSimple.Direction.REVERSE);
        fr = setupMotor("fr", DcMotorSimple.Direction.FORWARD);
        br = setupMotor("br", DcMotorSimple.Direction.FORWARD);

        // Outtake
        outtake = setupMotor("outtake", DcMotorSimple.Direction.FORWARD);
        outtake2 = setupMotor("outtake2", DcMotorSimple.Direction.FORWARD);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake
        frontIntake = setupMotor("frontIntake", DcMotorSimple.Direction.FORWARD);
        backIntake = setupMotor("backIntake", DcMotorSimple.Direction.FORWARD);

        /*
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        shroud = hardwareMap.get(Servo.class, "shroudCont");
        shroud.setPosition(shroudPos);


        // Sensors
        lemon = hardwareMap.get(Limelight3A.class, "limelight");
        lemon.pipelineSwitch(0);
        lemon.setPollRateHz(100);
        lemon.start();
        */

        time = new ElapsedTime();

        initIMU();

        previousTime = getRuntime();
    }

    @Override
    public void loop() {
        handleOuttakeControls();
        handleIntakeControls();
        handleDrive();
        //handleLimelight();
        //handleAprilTagAlign();
        telemetry.addData("Target-Velocity", targetVelocity);

        telemetry.update();
    }

    private void handleOuttakeControls() {
        if (gamepad2.right_trigger>0.1) {
            p = 0;
            i = 0;
            d = 0;


            currentVelocity = outtake.getVelocity();
            //if (Math.abs(targetVelocity - currentVelocity) > specialtolerance) {
            if ((Math.abs(targetVelocity - currentVelocity) > specialtolerance)){
                currentTime = time.milliseconds();
                error = targetVelocity - currentVelocity;


                if (targetVelocity >= currentVelocity){
                    p = specialKP * error;
                    i += (specialKI * (error * (currentTime - previousTime)));
                    i = Range.clip(i, -20, 20);
                    d = specialKD * ((error - previousError) / (currentTime - previousTime));
                    outtakeVelocity = p + i + d;
                }


                previousError = error;
                previousTime = currentTime;
                currentVelocity = outtake.getVelocity();
                outtakeVelocity = Range.clip(outtakeVelocity,0,2000);
                outtake.setVelocity(outtakeVelocity);
                outtake2.setVelocity(outtakeVelocity);


            }
            outtake.setVelocity(outtakeVelocity);
            outtake2.setVelocity(outtakeVelocity);
        }
        else{
            outtake.setPower(0);
            outtake2.setPower(0);
        }
        if (gamepad2.dpad_up && !gamepad2DpadUpWasPressed) {
            targetVelocity += 10;
        }
        gamepad2DpadUpWasPressed = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !gamepad2DpadDownWasPressed) {
            targetVelocity -= 10;
        }
        gamepad2DpadDownWasPressed= gamepad2.dpad_down;
        if(gamepad2.left_stick_button){
            targetVelocity=1200;
        }
        else if(gamepad2.right_stick_button){
            targetVelocity=1550;
        }
    }

    private void handleIntakeControls(){
        if (gamepad2.left_bumper){
            frontIntake.setPower(1);
            backIntake.setPower(-1);
        }else if (gamepad2.right_bumper) {
            frontIntake.setPower(1);
            backIntake.setPower(1);
        }else if(gamepad2.b){
            frontIntake.setPower(1);
            backIntake.setPower(0);
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
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Check for Driver 1 "B" button for Auto-Lock
        if (gamepad1.b) {
            rx = -handleAprilTagAlign();
        }

        double flPwr = y + x + rx;
        double blPwr = y - x + rx;
        double frPwr = y - x - rx;
        double brPwr = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(flPwr), Math.max(Math.abs(blPwr),
                Math.max(Math.abs(frPwr), Math.abs(brPwr)))));

        fl.setPower(flPwr / max);
        bl.setPower(blPwr / max);
        fr.setPower(frPwr / max);
        br.setPower(brPwr / max);
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
    private final double ALIGN_KP = 0.06; // Proportional gain: adjust if it oscillates or is too slow
    private final double ALIGN_TOLERANCE = 0.5; // Degrees of error allowed

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