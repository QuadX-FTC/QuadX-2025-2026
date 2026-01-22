package org.firstinspires.ftc.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
@Autonomous(name = "XTestX", group = "Autonomous")
public class BlueCloseAutoWithLimelightAndPID extends LinearOpMode {

    // ==================== TUNABLE PARAMETERS ====================
    public static double SHOOTER_TARGET_RPM = 2500;
    public static double SHOOTER_MAX_POWER = 1.0;
    public static double SHOOTER_SPINUP_TIMEOUT = 3.0;
    public static double SHOOTER_SHOOT_DURATION = 1.5;

    public static double INTAKE_CURRENT_THRESHOLD = 2.0;
    public static double INTAKE_MAX_TIME = 2.5;

    public static double LIMELIGHT_ALIGN_TIMEOUT = 2.0;
    public static double LIMELIGHT_MAX_POWER = 0.3;
    public static double LIMELIGHT_TOLERANCE = 1.0;

    // PID Coefficients for Shooter Velocity
    public static double SHOOTER_Kp = 0.002;
    public static double SHOOTER_Ki = 0.00005;
    public static double SHOOTER_Kd = 0.0001;

    // PID Coefficients for Limelight Alignment
    public static double LIMELIGHT_Kp = 0.02;
    public static double LIMELIGHT_Ki = 0.005;
    public static double LIMELIGHT_Kd = 0.005;

    // ==================== SMART INTAKE CLASS ====================
    public class SmartIntake {
        private DcMotorEx intakeFront;
        private DcMotorEx intakeBack;

        public SmartIntake(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
            intakeFront = hardwareMap.get(DcMotorEx.class, "frontIntake");
            intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);

            intakeBack = hardwareMap.get(DcMotorEx.class, "backIntake");
            intakeBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Version 1: Smart intake with current sensing (no duration)
        public class SmartIntakeAction implements Action {
            private final double powerFront;
            private final double powerBack;
            private ElapsedTime runtime = new ElapsedTime();
            private boolean initialized = false;
            private boolean pieceDetected = false;

            public SmartIntakeAction(double powerFront, double powerBack) {
                this.powerFront = powerFront;
                this.powerBack = powerBack;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    runtime.reset();
                    intakeFront.setPower(powerFront);
                    intakeBack.setPower(powerBack);
                    initialized = true;
                }

                double currentFront = intakeFront.getCurrent(CurrentUnit.AMPS);
                double currentBack = intakeBack.getCurrent(CurrentUnit.AMPS);
                double avgCurrent = (currentFront + currentBack) / 2.0;

                packet.put("[INTAKE] Front Current (A)", String.format("%.2f", currentFront));
                packet.put("[INTAKE] Back Current (A)", String.format("%.2f", currentBack));
                packet.put("[INTAKE] Avg Current (A)", String.format("%.2f", avgCurrent));
                packet.put("[INTAKE] Time (s)", String.format("%.2f", runtime.seconds()));

                if (avgCurrent > INTAKE_CURRENT_THRESHOLD) {
                    pieceDetected = true;
                }

                if ((pieceDetected && runtime.seconds() > 0.3) || runtime.seconds() > INTAKE_MAX_TIME) {
                    intakeFront.setPower(0);
                    intakeBack.setPower(0);
                    packet.put("[INTAKE] Status", pieceDetected ? "GRABBED" : "TIMEOUT");
                    return false;
                }

                return true;
            }
        }

        // Version 2: Time-based intake (like original)
        public class TimedIntakeAction implements Action {
            private final double powerFront;
            private final double powerBack;
            private final double durationSeconds;
            private ElapsedTime runtime = new ElapsedTime();
            private boolean initialized = false;

            public TimedIntakeAction(double powerFront, double powerBack, double durationSeconds) {
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

                packet.put("[INTAKE TIMED] Time (s)", String.format("%.2f", runtime.seconds()));

                if (runtime.seconds() < durationSeconds) {
                    return true;
                } else {
                    intakeFront.setPower(0);
                    intakeBack.setPower(0);
                    packet.put("[INTAKE TIMED] Status", "DONE");
                    return false;
                }
            }
        }

        // Smart intake (current-based, no duration)
        public Action smartIntake(double powerFront, double powerBack) {
            return new SmartIntakeAction(powerFront, powerBack);
        }

        // Timed intake (duration-based)
        public Action smartIntake(double powerFront, double powerBack, double durationSeconds) {
            return new TimedIntakeAction(powerFront, powerBack, durationSeconds);
        }
    }

    // ==================== SHOOTER VELOCITY PID CLASS ====================
    public class ShooterVelocity {
        private DcMotorEx outtake1;
        private DcMotorEx outtake2;

        private double integral = 0;
        private double previousError = 0;

        public ShooterVelocity(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
            outtake1 = hardwareMap.get(DcMotorEx.class, "outtake");
            outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // SpinUp action: bring shooter to target RPM with PID
        public class SpinUpAction implements Action {
            private final double targetRPM;
            private ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            public SpinUpAction(double targetRPM) {
                this.targetRPM = targetRPM;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    integral = 0;
                    previousError = 0;
                    initialized = true;
                }

                // Get current RPM from motor encoders
                double currentRPM1 = outtake1.getVelocity() / 360.0 * 60.0;
                double currentRPM2 = outtake2.getVelocity() / 360.0 * 60.0;
                double avgRPM = (currentRPM1 + currentRPM2) / 2.0;

                // Calculate error
                double error = targetRPM - avgRPM;

                // PID calculation
                double p = SHOOTER_Kp * error;

                integral += error;
                integral = Range.clip(integral, -100, 100);
                double i = SHOOTER_Ki * integral;

                double d = SHOOTER_Kd * (error - previousError);

                // Total power output
                double power = p + i + d;
                power = Range.clip(power, -SHOOTER_MAX_POWER, SHOOTER_MAX_POWER);

                outtake1.setPower(power);
                outtake2.setPower(power);

                // Telemetry
                packet.put("[SHOOTER SPINUP] Target RPM", (int)targetRPM);
                packet.put("[SHOOTER SPINUP] Avg RPM", (int)avgRPM);
                packet.put("[SHOOTER SPINUP] Error", (int)error);
                packet.put("[SHOOTER SPINUP] Power", String.format("%.3f", power));
                packet.put("[SHOOTER SPINUP] Time (s)", String.format("%.2f", timer.seconds()));

                previousError = error;

                // Success: reached target RPM (within 50 RPM tolerance)
                if (Math.abs(error) < 50) {
                    packet.put("[SHOOTER SPINUP] Status", "READY");
                    return false;
                }

                // Timeout safety
                if (timer.seconds() > SHOOTER_SPINUP_TIMEOUT) {
                    packet.put("[SHOOTER SPINUP] Status", "TIMEOUT");
                    return false;
                }

                return true;
            }
        }

        // Shoot action: maintain RPM while shooting
        public class ShootAction implements Action {
            private final double targetRPM;
            private final double durationSeconds;
            private ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            public ShootAction(double targetRPM, double durationSeconds) {
                this.targetRPM = targetRPM;
                this.durationSeconds = durationSeconds;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    integral = 0;
                    previousError = 0;
                    initialized = true;
                }

                double currentRPM1 = outtake1.getVelocity() / 360.0 * 60.0;
                double currentRPM2 = outtake2.getVelocity() / 360.0 * 60.0;
                double avgRPM = (currentRPM1 + currentRPM2) / 2.0;

                double error = targetRPM - avgRPM;

                double p = SHOOTER_Kp * error;
                integral += error;
                integral = Range.clip(integral, -100, 100);
                double i = SHOOTER_Ki * integral;
                double d = SHOOTER_Kd * (error - previousError);

                double power = p + i + d;
                power = Range.clip(power, -1.0, 1.0);

                outtake1.setPower(power);
                outtake2.setPower(power);

                packet.put("[SHOOTER SHOOT] Target RPM", (int)targetRPM);
                packet.put("[SHOOTER SHOOT] Avg RPM", (int)avgRPM);
                packet.put("[SHOOTER SHOOT] Power", String.format("%.3f", power));
                packet.put("[SHOOTER SHOOT] Time (s)", String.format("%.2f", timer.seconds()));

                previousError = error;

                // Stop after duration
                if (timer.seconds() > durationSeconds) {
                    outtake1.setPower(0);
                    outtake2.setPower(0);
                    packet.put("[SHOOTER SHOOT] Status", "DONE");
                    return false;
                }

                return true;
            }
        }

        public Action spinUp(double targetRPM) {
            return new SpinUpAction(targetRPM);
        }

        public Action shoot(double targetRPM, double duration) {
            return new ShootAction(targetRPM, duration);
        }
    }

    // ==================== LIMELIGHT ALIGNMENT CLASS ====================
    public class LimelightAligner {
        private Limelight3A limelight;
        private MecanumDrive drive;

        private double integral = 0;
        private double previousError = 0;

        public LimelightAligner(Limelight3A limelight, MecanumDrive drive) {
            this.limelight = limelight;
            this.drive = drive;
        }

        public class AlignAction implements Action {
            private final double targetTx;
            private ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            public AlignAction(double targetTx) {
                this.targetTx = targetTx;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    integral = 0;
                    previousError = 0;
                    initialized = true;
                }

                double tx = limelight.getLatestResult().getTx();
                boolean tv = limelight.getLatestResult().isValid();

                packet.put("[LIMELIGHT ALIGN] Target Visible", tv);
                packet.put("[LIMELIGHT ALIGN] TX (degrees)", String.format("%.2f", tx));
                packet.put("[LIMELIGHT ALIGN] Time (s)", String.format("%.2f", timer.seconds()));

                // No target found
                if (!tv) {
                    packet.put("[LIMELIGHT ALIGN] Status", "NO TARGET");
                    return timer.seconds() < LIMELIGHT_ALIGN_TIMEOUT;
                }

                // Calculate error
                double error = targetTx - tx;

                packet.put("[LIMELIGHT ALIGN] Error", String.format("%.2f", error));

                // Success: within tolerance
                if (Math.abs(error) < LIMELIGHT_TOLERANCE) {
                    packet.put("[LIMELIGHT ALIGN] Status", "ALIGNED");
                    return false;
                }

                // Timeout
                if (timer.seconds() > LIMELIGHT_ALIGN_TIMEOUT) {
                    packet.put("[LIMELIGHT ALIGN] Status", "TIMEOUT");
                    return false;
                }

                return true;
            }
        }

        public Action align(double targetTx) {
            return new AlignAction(targetTx);
        }
    }

    // ==================== MAIN AUTONOMOUS ====================
    @Override
    public void runOpMode() {
        // Initialize starting pose
        Pose2d initialPose = new Pose2d(-56, -46, Math.toRadians(180));

        telemetry.addLine("========== DECODE 9-BALL AUTO INITIALIZATION ==========");
        telemetry.update();

        // Initialize Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize core components
        SmartIntake intake = new SmartIntake(hardwareMap);
        ShooterVelocity shooter = new ShooterVelocity(hardwareMap);

        // Initialize Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        LimelightAligner aligner = new LimelightAligner(limelight, drive);

        // Build Road Runner trajectories (matching original code coordinates)
        // PRELOAD: Start at (-56, -46), back up and right to goal at (-19.5, -24)
        TrajectoryActionBuilder toGoal = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(-19.5, -24), Math.toRadians(270));

        // CYCLE 1: From goal to Pile 1
        TrajectoryActionBuilder traj1 = toGoal.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-45, -25), Math.toRadians(230));

        // CYCLE 1: From Pile 1, drive down to prep zone
        TrajectoryActionBuilder traj3 = traj1.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-19.5, -46), Math.toRadians(270));

        // CYCLE 1: From prep zone back up to goal
        TrajectoryActionBuilder traj2 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-19.5, -24), Math.toRadians(270));

        // CYCLE 2: From goal to Pile 2
        TrajectoryActionBuilder traj4 = traj2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-43, -28), Math.toRadians(225));

        // CYCLE 2: From Pile 2, drive down and around to prep zone
        TrajectoryActionBuilder traj6 = traj4.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(2, -56), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(2, -42), Math.toRadians(270));

        // CYCLE 2: From prep zone back up to goal
        TrajectoryActionBuilder traj5 = traj6.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(2, -24), Math.toRadians(270));

        // SAFE ZONE: From goal to safe zone
        Action trajectoryActionCloseOut = traj5.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-43, -15), Math.toRadians(240))
                .build();

        // Setup hardware
        while (!isStopRequested() && !opModeIsActive()) {
            Servo shroud = hardwareMap.get(Servo.class, "shroudCont");
            shroud.setPosition(0.5);
            telemetry.addData("Shooter Target RPM", SHOOTER_TARGET_RPM);
            telemetry.addData("Intake Current Threshold", INTAKE_CURRENT_THRESHOLD);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("AUTO STARTED");
        telemetry.update();

        // ==================== AUTONOMOUS SEQUENCE - 9 BALL (3 PRELOAD + 2 PILES) ====================
        Actions.runBlocking(
                new SequentialAction(
                        // ===== PRELOAD PHASE: Drive to goal, then shoot 3 balls =====
                        // Drive back and to the right to align with goal
                        toGoal.build(),

                        // Ball 1: Spinup and shoot
                        shooter.spinUp(SHOOTER_TARGET_RPM),
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // Ball 2: Already loaded, just shoot (spinup maintained)
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // Ball 3: Already loaded, just shoot
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // ===== CYCLE 1: Pile 1 - Intake 3 balls, return to goal, shoot 3 =====
                        // Drive to Pile 1
                        traj1.build(),

                        // Intake Ball 1 from Pile 1
                        intake.smartIntake(1, 1, 1),       // Push ball up ramp
                        intake.smartIntake(-0.3, 1, 1),    // Shuffle back
                        intake.smartIntake(1, 1, 1.7),     // Load into shooter

                        // Intake Ball 2 from Pile 1
                        intake.smartIntake(1, 1, 1),       // Push ball up ramp
                        intake.smartIntake(-0.3, 1, 1),    // Shuffle back
                        intake.smartIntake(1, 1, 1.7),     // Load into shooter

                        // Intake Ball 3 from Pile 1
                        intake.smartIntake(1, 1, 1),       // Push ball up ramp
                        intake.smartIntake(-0.3, 1, 1),    // Shuffle back
                        intake.smartIntake(1, 1, 1.7),     // Load into shooter

                        // Drive back to prep zone
                        traj3.build(),

                        // Drive from prep zone back up to goal
                        traj2.build(),

                        // Shoot Ball 1 from Pile 1
                        shooter.spinUp(SHOOTER_TARGET_RPM),
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // Shoot Ball 2 from Pile 1
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // Shoot Ball 3 from Pile 1
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // ===== CYCLE 2: Pile 2 - Intake 3 balls, return to goal, shoot 3 =====
                        // Drive to Pile 2
                        traj4.build(),

                        // Intake Ball 1 from Pile 2
                        intake.smartIntake(1, 1, 1),       // Push ball up ramp
                        intake.smartIntake(-0.3, 1, 1),    // Shuffle back
                        intake.smartIntake(1, 1, 1.7),     // Load into shooter

                        // Intake Ball 2 from Pile 2
                        intake.smartIntake(1, 1, 1),       // Push ball up ramp
                        intake.smartIntake(-0.3, 1, 1),    // Shuffle back
                        intake.smartIntake(1, 1, 1.7),     // Load into shooter

                        // Intake Ball 3 from Pile 2
                        intake.smartIntake(1, 1, 1),       // Push ball up ramp
                        intake.smartIntake(-0.3, 1, 1),    // Shuffle back
                        intake.smartIntake(1, 1, 1.7),     // Load into shooter

                        // Drive back to prep zone (complex path down and around)
                        traj6.build(),

                        // Drive from prep zone back up to goal
                        traj5.build(),

                        // Shoot Ball 1 from Pile 2
                        shooter.spinUp(SHOOTER_TARGET_RPM),
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // Shoot Ball 2 from Pile 2
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // Shoot Ball 3 from Pile 2
                        aligner.align(0),
                        shooter.shoot(SHOOTER_TARGET_RPM, SHOOTER_SHOOT_DURATION),

                        // ===== SAFE ZONE: Drive to safe zone =====
                        trajectoryActionCloseOut
                )
        );

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }
}
