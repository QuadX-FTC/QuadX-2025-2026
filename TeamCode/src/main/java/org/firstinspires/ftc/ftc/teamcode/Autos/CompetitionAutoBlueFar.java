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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "CompetitionAutoBlueFar", group = "Autonomous")
public class CompetitionAutoBlueFar extends LinearOpMode {

    public class WaitAction implements Action {
        private final double durationSeconds;
        private ElapsedTime runtime = new ElapsedTime();
        private boolean initialized = false;

        public WaitAction(double durationSeconds) {
            this.durationSeconds = durationSeconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                runtime.reset();
                initialized = true;
            }

            if (runtime.seconds() < durationSeconds) {
                return true;
            } else {
                return false;
            }
        }
    }

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
                    runtime.reset();
                    intakeFront.setPower(powerFront);
                    intakeBack.setPower(powerBack);
                    initialized = true;
                }

                if (runtime.seconds() < durationSeconds) {
                    return true;
                } else {
                    intakeFront.setPower(0);
                    intakeBack.setPower(0);
                    return false;
                }
            }
        }

        public Action runForDuration(double powerFront, double powerBack, double durationSeconds) {
            return new RunIntakeForTime(powerFront, powerBack, durationSeconds);
        }
    }

    public class ShooterPID {
        private DcMotorEx outtake1;
        private DcMotorEx outtake2;

        // PID Constants
        public double specialKP = 0.0;
        public double specialKI = 0.0;
        public double specialKD = 0.0;
        public double specialTolerance = 10; // velocity error tolerance

        // PID Variables
        private double targetVelocity = 0;
        private double currentVelocity = 0;
        private double error = 0;
        private double previousError = 0;
        private double p = 0, i = 0, d = 0;
        private double outtakeVelocity = 0;

        private ElapsedTime time = new ElapsedTime();
        private double currentTime = 0;
        private double previousTime = 0;
        private boolean isRunning = false;

        public ShooterPID(HardwareMap hardwareMap) {
            outtake1 = hardwareMap.get(DcMotorEx.class, "outtake");
            outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtake1.setDirection(DcMotorSimple.Direction.REVERSE);

            outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

            time.reset();
            previousTime = 0;
        }

        public class RunShooterPID implements Action {
            private final double targetVel;
            private boolean initialized = false;

            public RunShooterPID(double targetVelocity) {
                this.targetVel = targetVelocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setTargetVelocity(targetVel);
                    isRunning = true;
                    initialized = true;
                }
                updatePID();
                return true; // Runs indefinitely until explicitly stopped
            }
        }

        private void updatePID() {
            if (!isRunning) return;

            currentVelocity = outtake1.getVelocity();

            if (Math.abs(targetVelocity - currentVelocity) > specialTolerance) {
                currentTime = time.milliseconds();
                error = targetVelocity - currentVelocity;

                if (targetVelocity >= currentVelocity) {
                    p = specialKP * error;
                    i += (specialKI * (error * (currentTime - previousTime)));
                    i = Range.clip(i, -20, 20);
                    d = specialKD * ((error - previousError) / (currentTime - previousTime));
                    outtakeVelocity = p + i + d;
                }

                previousError = error;
                previousTime = currentTime;
                outtakeVelocity = Range.clip(outtakeVelocity, 0, 2000);
            }

            outtake1.setVelocity(outtakeVelocity);
            outtake2.setVelocity(outtakeVelocity);
        }

        public void setTargetVelocity(double velocity) {
            targetVelocity = velocity;
        }

        public void stopShooter() {
            isRunning = false;
            targetVelocity = 0;
            outtake1.setVelocity(0);
            outtake2.setVelocity(0);
        }

        public Action startShooter(double targetVelocity) {
            return new RunShooterPID(targetVelocity);
        }

        public double getTargetVelocity() {
            return targetVelocity;
        }

        public double getCurrentVelocity() {
            return currentVelocity;
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(60, -10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        ShooterPID shooter = new ShooterPID(hardwareMap);

        shooter.specialKP = 24;
        shooter.specialKI = 100;
        shooter.specialKD = 110;
        shooter.specialTolerance = 10;

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(55, -10), Math.toRadians(210));

        TrajectoryActionBuilder traj2 =  traj1.endTrajectory().fresh()
                .lineToXLinearHeading(32, Math.toRadians(270));

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(32, -57), Math.toRadians(270));

        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(55, -10), Math.toRadians(200));

        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(7.5, -10), Math.toRadians(270));

        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(7.5, -57), Math.toRadians(270));

        Action trajectoryActionCloseOut = traj6.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(55, -10), Math.toRadians(201))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            Servo shroud = hardwareMap.get(Servo.class, "shroudCont");
            shroud.setPosition(0.5);
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Shooter Target", shooter.getTargetVelocity());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                shooter.startShooter(1525),
                                new SequentialAction(
                                        new WaitAction(2.2),
                                        traj1.build(),
                                        intake.runForDuration(1, 1, 1),
                                        intake.runForDuration(-0.3, 1, 1),
                                        intake.runForDuration(1, 1, 1.7),
                                        traj2.build(),
                                        new ParallelAction(
                                                traj3.build(),
                                                intake.runForDuration(1, -0.7, 2)
                                        ),
                                        traj4.build(),
                                        intake.runForDuration(1, 1, 1),
                                        intake.runForDuration(-0.3, 1, 1),
                                        intake.runForDuration(1, 1, 1.7),
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
                        )
                )
        );

        shooter.stopShooter();
    }
}