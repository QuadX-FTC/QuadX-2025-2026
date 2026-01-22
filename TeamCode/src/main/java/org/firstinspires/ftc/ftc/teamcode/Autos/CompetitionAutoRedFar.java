package org.firstinspires.ftc.ftc.teamcode.Autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "CompetitionAutoRedClose", group = "Autonomous")
public class CompetitionAutoRedFar extends LinearOpMode {


    //Outtake pid = new Outtake();


    public class Intake {
        private DcMotorEx intakeFront;
        private DcMotorEx intakeBack;


        public Intake(HardwareMap hardwareMap) {
            intakeFront = hardwareMap.get(DcMotorEx.class, "frontIntake");
            intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeBack = hardwareMap.get(DcMotorEx.class, "backIntake");
            intakeBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class RunIntakeForTime implements Action {
            private final double durationSeconds;
            private final double powerFront;
            private final double powerBack;
            private ElapsedTime runtime = new ElapsedTime();
            private boolean initialized = false;


            public RunIntakeForTime(double powerFront, double powerBack, double durationSeconds) {
                this.powerFront = powerFront;
                this.powerBack = powerBack;
                this.durationSeconds = durationSeconds;
            }


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    runtime.reset(); // Start the timer
                    intakeFront.setPower(powerFront);
                    intakeBack.setPower(powerBack);
                    initialized = true;
                }


                // Check if the elapsed time is less than the desired time
                if (runtime.seconds() < durationSeconds) {
                    return true; // Continue running
                } else {
                    intakeFront.setPower(0); // Stop the motors
                    intakeBack.setPower(0);
                    return false; // Stop running
                }
            }
        }
        public Action runForDuration(double powerFront, double powerBack, double durationSeconds) {
            return new RunIntakeForTime(powerFront, powerBack, durationSeconds);
        }
    }


    public class Outtake1 {
        private DcMotorEx outtake1;
        private DcMotorEx outtake2;


        public Outtake1 (HardwareMap hardwareMap){
            outtake1 = hardwareMap.get(DcMotorEx.class, "outtake");
            outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class RunOuttakeForTime implements Action {
            private final double durationSeconds;
            private final double power;
            private ElapsedTime runtime = new ElapsedTime();
            private boolean initialized = false;


            public RunOuttakeForTime(double power, double durationSeconds) {
                this.power = power;
                this.durationSeconds = durationSeconds;
            }


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    runtime.reset(); // Start the timer
                    outtake1.setPower(power);
                    outtake2.setPower(power);
                    initialized = true;
                }


                // Check if the elapsed time is less than the desired time
                if (runtime.seconds() < durationSeconds) {
                    return true; // Continue running
                } else {
                    //outtake1.setPower(0); // Stop the motors
                    //outtake2.setPower(0);
                    return false; // Stop running
                }
            }
        }
        public Action runForDuration(double power, double durationSeconds) {
            return new RunOuttakeForTime(power, durationSeconds);
        }
    }




    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-56, 46, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Outtake1 shooter = new Outtake1(hardwareMap);


        // vision here that outputs pattern (Do later)
        // int visionOutputPosition = 1;


        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(-45, 25), Math.toRadians(130));


        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-18, 15), Math.toRadians(90));


        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-18, 46), Math.toRadians(90));


        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-43, 28), Math.toRadians(135));


        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(3, 15), Math.toRadians(90));


        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(3, 56), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(3, 42), Math.toRadians(90));


        Action trajectoryActionCloseOut = traj6.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-43, 15), Math.toRadians(120))
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
            Servo shroud = hardwareMap.get(Servo.class, "shroudCont");
            shroud.setPosition(0.5);
            //int position = visionOutputPosition;
            //telemetry.addData("Position during Init", position);
            //telemetry.update();
        }


        //int startPosition = visionOutputPosition;
        //telemetry.addData("Starting Position", startPosition);
        //telemetry.update();
        waitForStart();


        if (isStopRequested()) return;


       /*
       Action trajectoryActionChosen;
       if (startPosition == 1) {
           trajectoryActionChosen = tab1.build();
       } else if (startPosition == 2) {
           trajectoryActionChosen = tab2.build();
       } else {
           trajectoryActionChosen = tab3.build();
       }
       */


        Actions.runBlocking(
                new SequentialAction(
                        shooter.runForDuration(0.518,1.5),
                        traj1.build(),
                        intake.runForDuration(1,1,1),
                        intake.runForDuration(-0.3,1,1),
                        intake.runForDuration(1,1,1.7),
                        traj2.build(),
                        new ParallelAction(
                                traj3.build(),
                                intake.runForDuration(1,-0.7,2)
                        ),
                        traj4.build(),
                        intake.runForDuration(1,1,1),
                        intake.runForDuration(-0.3,1,1),
                        intake.runForDuration(1,1,1.7),
                        traj5.build(),
                        new ParallelAction(
                                traj6.build(),
                                intake.runForDuration(1,-0.7,2)
                        ),
                        trajectoryActionCloseOut,
                        intake.runForDuration(1,1,1),
                        intake.runForDuration(-0.3,1,1),
                        intake.runForDuration(1,1,2),
                        traj3.build()
                )
        );
    }
}
