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

@TeleOp(name="CompetitionTeleOpRed", group="TeleOp")
public class CompetitionTeleOpRed extends OpMode{
    private DcMotorEx outtake, outtake2, fl, fr, bl, br, frontIntake, backIntake;
    private Servo shroud;

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
    double motorVelocity;

    double currentVelocity;

    // control state variables for toggles
    boolean gamepad1DpadUpWasPressed = false;
    boolean gamepad1DpadDownWasPressed = false;
    boolean gamepad2DpadUpWasPressed = false;
    boolean gamepad2DpadDownWasPressed = false;
    boolean gamepad2DpadLeftWasPressed = false;
    boolean gamepad2DpadRightWasPressed = false;

    boolean gamepad2DpadBackWasPressed = false;

    // init states for outtake and shroud
    double targetOuttakePwr = 0.8;
    double pos = 0.5;
    // outtake toggle
    boolean outtakeOn = false;
    boolean toggle = false;
    double specialKP;
    double specialKI;
    double specialKD;
    double specialtolerance;
    double outtakeVelocity;
    double targetVelocity = 1500;

    double p;
    double i;
    double d;




    @Override
    public void init() {

        time = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shroud = hardwareMap.get(Servo.class, "shroudCont");
        shroud.setPosition(pos); // Set initial position

        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int[] validIDs = {20,24};
        Lemon = hardwareMap.get(Limelight3A.class,"limelight");
        Lemon.setPollRateHz(100); //this is how many times we ask the Limelight for information PER SECOND.
        Lemon.start();
        Lemon.pipelineSwitch(1);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        tolerance = 0.63;
        previousTime = 0;
        previousError = 0;
        Kp = 0.15;
        Ki = 0.05;
        Kd = 0.3;
        max_i = 0.2;
        min_i = -0.2;
        motorPower = 0;
        currentTime = 0;
        error = 0;
        specialKP = 24; //Re-tune testing = 15
        specialKI = 100;
        specialKD = 110;
        specialtolerance = 0;

        telemetry.setMsTransmissionInterval(10);
    }

    @Override
    public void loop() {
        // Outtake power toggle
        // dpad up to increase by 5%, dpad down to decrease by 5%
        if (gamepad1.dpad_up && !gamepad1.dpad_up) {
            targetOuttakePwr += 0.05; // Increment by 5%
            targetOuttakePwr = Range.clip(targetOuttakePwr, 0.0, 1.0);
            telemetry.addData("Outtake Target Power", targetOuttakePwr);
            telemetry.update();
        }

        if (gamepad1.dpad_down && !gamepad1.dpad_down) {
            targetOuttakePwr -= 0.05; // Decrement by 5%
            targetOuttakePwr = Range.clip(targetOuttakePwr, 0.0, 1.0);
            telemetry.addData("Outtake Target Power", targetOuttakePwr);
            telemetry.update();
        }

        //Outtake
        //right trigger to turn on shooter, left trigger to turn on shooter (no need to hold down trigger)
        if(gamepad2.right_trigger > 0.05){
            outtakeOn = true;
            telemetry.addData("shooter state", "ON");
        }
        if(gamepad2.left_trigger > 0.05){
            outtakeOn = false;
            telemetry.addData("shooter state", "OFF");
        }
        if(outtakeOn){
            outtake.setPower(targetOuttakePwr);
            outtake2.setPower(targetOuttakePwr);
        }
        if(!outtakeOn) {
            outtake.setPower(0);
            outtake2.setPower(0);
        }

        //Drive
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;
        

        LLResult results = Lemon.getLatestResult();

        if (!(results == null) && results.isValid()){
            tx = results.getTx(); // How far left or right the target is (degrees)
            double ty = results.getTy(); // How far up or down the target is (degrees)
            double ta = results.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else{
            telemetry.addData("Limelight: ","No Targets");
        }

        if (results != null) {
            if (results.isValid()) {
                Pose3D botpose = results.getBotpose();
                telemetry.addData("tx", results.getTx());
                telemetry.addData("ty", results.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }
        }

        double frPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double flPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double brPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double blPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        fr.setPower(frPower);
        fl.setPower(flPower);
        br.setPower(brPower);
        bl.setPower(blPower);

        if (gamepad1.b && results.isValid()) {
            double targetHeading = 0;
            p = 0;
            i = 0;
            d = 0;

            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            double heading = results.getTx();

            if ((Math.abs(targetHeading - heading) > tolerance) && !(targetVelocity + 40 < currentVelocity)){
                currentTime = time.milliseconds();
                error = targetHeading - heading;
                p = (Kp * error);
                i += (Ki * (error * (currentTime - previousTime)));
                i = Range.clip(i, min_i, max_i);
                d = (Kd * (error - previousError) / (currentTime - previousTime));

                motorPower = (p + i + d);

                previousError = error;
                previousTime = currentTime;
                heading = results.getTx();
                fl.setPower(motorPower);
                fr.setPower(motorPower * -1);
                bl.setPower(motorPower);
                br.setPower(motorPower * -1);
            }
        }

        if(gamepad2.left_stick_button){
            targetVelocity = 1500;
        }

        if ((gamepad2.back) && (toggle)){
            toggle = false;
        }

        if (!(toggle)) {
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

            toggle = true;
        }

        //Intake
        if (gamepad2.right_bumper) {
            frontIntake.setPower(1);
            backIntake.setPower(1);
        } else if (gamepad2.left_bumper) {
            frontIntake.setPower(-1);
            backIntake.setPower(-1);
        } else if (gamepad2.a) {
            frontIntake.setPower(1);
        }else if (gamepad2.b) {
            backIntake.setPower(-0.6);
            frontIntake.setPower(1);
        } else if (gamepad2.x) {
            frontIntake.setPower(-1);
        }else if (gamepad2.y) {
            backIntake.setPower(-1);
        } else {
            frontIntake.setPower(0);
            backIntake.setPower(0);
        }

        // Shroud
        if (gamepad2.dpad_up && !gamepad2DpadUpWasPressed) {
            pos += 0.1;
            pos = Range.clip(pos, 0.0, 1.0);
            shroud.setPosition(pos);
            telemetry.addData("shroud pos", pos);
        }
        gamepad2DpadUpWasPressed = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !gamepad2DpadDownWasPressed) {
            pos -= 0.1;
            pos = Range.clip(pos, 0.0, 1.0);
            shroud.setPosition(pos);
            telemetry.addData("shroud pos", pos);
        }
        gamepad2DpadDownWasPressed= gamepad2.dpad_down;

        //changing of the target velocity for the PID. Increments are 20 each, subject to change.
        if (gamepad2.dpad_left && !gamepad2DpadLeftWasPressed){
            targetVelocity = targetVelocity - 20;
        }
        gamepad2DpadLeftWasPressed = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !gamepad2DpadRightWasPressed){
            targetVelocity = targetVelocity + 20;
        }
        gamepad2DpadRightWasPressed = gamepad2.dpad_right;

        telemetry.addData("TargetVelocity: ",targetVelocity);
        telemetry.addData("Outtake Velocity: ", outtake.getPower());
        telemetry.addData("Outtake2 Velocity: ", outtake2.getPower());
        telemetry.addData("Shooter Current Velocity", currentVelocity);
        telemetry.addData("Is PID Active", toggle);
        telemetry.addData("p",p);
        telemetry.addData("i",i);
        telemetry.addData("d",d);
        telemetry.addData("error",error);
        telemetry.update();
    }
}