package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PIDs.Drivetrain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class LimelightTeleOpwithDrivetrainMacros extends OpMode {
    private DcMotor fr, fl, br, bl;
    private boolean straightAssist = false;
    private boolean lastToggle = false;
    private HuskyLens Husky;
    private Limelight3A Lemon;
    private Drivetrain drivetrainMacros;
    double tx;
    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Husky = hardwareMap.get(HuskyLens.class,"Husky");
        Husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        int[] validIDs = {20,24};
        Lemon = hardwareMap.get(Limelight3A.class,"limelight");
        Lemon.setPollRateHz(100); //this is how many times we ask the Limelight for information PER SECOND.
        Lemon.start();

        Lemon.pipelineSwitch(0);
    }

    @Override
    public void loop() {

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



        if (gamepad1.a && !lastToggle) {
            straightAssist = !straightAssist;
        }
        lastToggle = gamepad1.a;

        float drive = -gamepad1.left_stick_y;
        float strafe = gamepad1.left_stick_x;
        float turn   = -gamepad1.right_stick_x;

        if (straightAssist) {
            drive = snapInput(drive);
            strafe = snapInput(strafe);
            turn = snapInput(turn);
        }

        if (gamepad1.b && results.isValid()) {
            drivetrainMacros.turn(tx);
        }

        double frPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double flPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double brPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double blPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        fr.setPower(frPower);
        fl.setPower(flPower);
        br.setPower(brPower);
        bl.setPower(blPower);

        telemetry.addData("Straight Assist", straightAssist ? "ON" : "OFF");
        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.update();

        TagDetection();
        TagCheck();

    }


    public HuskyLens.Block TagDetection(){
        int Num;
        HuskyLens.Block[] BlockList = Husky.blocks(1);
        Num = BlockList.length;
        if (Num >= 1){
            return(BlockList[0]);
        } else {
            return (null);
        }

    }

    public void TagCheck(){
        for (HuskyLens.Block b : Husky.blocks(1)) {
            if (b.id == 1){
                telemetry.addData("Full House",0);
            }

        }
    }


    private float snapInput(float val) {
        double round = 0.5;
        double round2 = 0.5;

        if (Math.abs(val) < round) {
            return 0;
        }
        if (Math.abs(val) > (1 - round2)) {
            if (val > 0) {
                return 1;
            } else {
                return -1;
            }
        }
        return val;
    }
}


