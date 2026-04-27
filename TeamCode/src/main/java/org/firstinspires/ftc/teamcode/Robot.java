package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainPP;
import org.firstinspires.ftc.teamcode.subsystems.Helidexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SpinBay;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.HoodAngle;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.livoniawarriors.GoBildaLedColors;
import org.livoniawarriors.LoggerCommandTimer;
import org.livoniawarriors.SequentialCommandGroup;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Robot {
    public static OpMode opMode;
    public static DrivetrainPP drivetrain;
    public static Controls controls;
    public static Intake intake;
    public static Turret turret;
    public static Shooter shooter;
    public static HoodAngle hoodAngle;
    public static Vision vision;
    private static Servo allianceLed;
    public static Helidexer helidexer;
    public static boolean IsRed = true;
    public static RobotTypeEnum RobotType = RobotTypeEnum.Helidexer;
    private static TelemetryPacket packet;

    private static LoggerCommandTimer logTimer;
    private static boolean isEnabled;
    private static boolean autonomous = false;
    public static boolean fromAuto = false;
    static int heliHome = 0;

    public enum RobotTypeEnum {
        Competition,
        Programming,
        Helidexer
    }

    public static void Init(OpMode inMode) {
        opMode = inMode;
        isEnabled = false;
        CommandScheduler.getInstance().setBulkReading(opMode.hardwareMap, LynxModule.BulkCachingMode.AUTO);
        packet = new TelemetryPacket();

        //if we have already run, clear out the old running subsystems
        if (drivetrain != null) {
            fromAuto = true;
            CommandScheduler.getInstance().unregisterSubsystem(drivetrain, intake,
                    shooter, hoodAngle, helidexer, turret, vision);
        }
        controls = new Controls();
        drivetrain = new DrivetrainPP();
        intake = new Intake();
        shooter = new Shooter();
        hoodAngle = new HoodAngle();
        helidexer = new Helidexer();
        turret = new Turret();
        vision = new Vision();

        allianceLed = opMode.hardwareMap.get(Servo.class, "RGB-Alliance");
        setLed();

        //we ran before, clear out all the old stuff running in the schedule
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(20);

        //buttons that run while disabled
        controls.flipAlliance().whenActive(flipAlliance());
        controls.resetTurretAngle().whenActive(turret.resetZero());
        controls.bumpHelidexerLeft().whenActive(helidexer.bumpHelidexer(-RobotConfig.POSITION_TOLERANCE));
        controls.bumpHelidexerRight().whenActive(helidexer.bumpHelidexer(RobotConfig.POSITION_TOLERANCE));
        controls.bumpTurretLeft().whenActive(turret.bumpTurretHome(-2));
        controls.bumpTurretRight().whenActive(turret.bumpTurretHome(2));

        logTimer = new LoggerCommandTimer("Shot Times");
        RobotState.initState();
        drivetrain.setPose(RobotState.robotPose);
    }

    public static void Periodic() {
        long startTime = System.nanoTime();
        loggingTime = 0;
        setLed();
        CommandScheduler.getInstance().run();

        double deltaTime = (System.nanoTime() - startTime) / 1_000_000.;
        packet.put("TimingMs/LoggingTime", loggingTime);
        packet.put("TimingMs/CommandTime", deltaTime);
        logPacket(packet);

        String alliance;
        if(IsRed) {
            alliance = "Red";
        } else {
            alliance = "Blue";
        }
        opMode.telemetry.addData("Alliance", alliance);
        opMode.telemetry.addData("TurretAngle", turret.getAngle());
        opMode.telemetry.addData("ShotDistance", drivetrain.getGoalDistance());
        opMode.telemetry.addData("Command TimeMs", deltaTime);
        opMode.telemetry.addData("_RPM Ready", shooter.atTarget());
        opMode.telemetry.addData("_Turret Ready", turret.atTarget());
        opMode.telemetry.addData("_Hood Ready", hoodAngle.atTarget());
        opMode.telemetry.update();
    }

    static double loggingTime;
    public static void logPacket(TelemetryPacket packet) {
        long startTime2 = System.nanoTime();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        loggingTime += (System.nanoTime() - startTime2) / 1_000_000.;
    }

    public static BooleanSupplier readyToShoot() {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return Robot.turret.atTarget() && Robot.shooter.atTarget() && Robot.hoodAngle.atTarget();
            }
        };
    }

    public static void scheduleTeleop() {
        //clear out auto
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        isEnabled = true;
        RobotState.initRun();

        //default commands that run when the robot is idle
        drivetrain.startTeleopDrive(true);
        drivetrain.setDefaultCommand(drivetrain.teleopDrive());
        turret.setDefaultCommand(turret.centerTurretViaPosition().perpetually());
        shooter.setDefaultCommand(shooter.autoShotRpm().perpetually());
        hoodAngle.setDefaultCommand(hoodAngle.autoShotHood().perpetually());

        //button commands
        controls.floorLoadActive().toggleWhenActive(commandFloorLoad());
        controls.resetFieldOriented().whenActive(drivetrain.resetFieldOriented());
        controls.shootActive().toggleWhenActive(shootAllBalls(false));
        //controls.bumpSpindexerLeft().whileActiveContinuous(helidexer.shootAll());
        //controls.bumpSpindexerRight().whileActiveContinuous(helidexer.advanceBay());
        controls.bumpHelidexerLeft().whenActive(helidexer.bumpHelidexer(-RobotConfig.POSITION_TOLERANCE));
        controls.bumpHelidexerRight().whenActive(helidexer.bumpHelidexer(RobotConfig.POSITION_TOLERANCE));
        controls.bumpTurretLeft().whenActive(turret.bumpTurretHome(-2));
        controls.bumpTurretRight().whenActive(turret.bumpTurretHome(2));
        controls.shootMotif().toggleWhenActive(shootAllBalls(true));
        controls.outtakeBalls().whileActiveContinuous(intake.runOuttake());
        controls.resetPoseTrigger().whileActiveOnce(resetRobotPose());

        controls.flipAlliance().whenActive(flipAlliance());
        controls.resetTurretAngle().whenActive(turret.resetZero());
        //controls.bumpTurretRotationsLeft().whenActive(turret.bumpRotations(-1));
        //controls.bumpTurretRotationsRight().whenActive(turret.bumpRotations(1));
        controls.shooterRpmDecrease().whenActive(()-> RobotState.shooterRpmAdjust -= 50);
        controls.shooterRpmIncrease().whenActive(()-> RobotState.shooterRpmAdjust += 50);

        //test commands
        Trigger shootCalibration = new Trigger(() -> Robot.opMode.gamepad1.start);
        shootCalibration.toggleWhenActive(calibrateShooter());
        /*shootCalibration.whileActiveContinuous(new ParallelDeadlineGroup(
                commandFloorLoad(),
                drivetrain.driveToBall()
        ));*/
    }

    public static void setAutonomous(Command autoSequence) {
        //clear out auto
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        CommandScheduler.getInstance().schedule(autoSequence);
    }

    public static void runAutonomous(LinearOpMode opMode, Supplier<Command> autoSequence, boolean isRed) {
        Robot.IsRed = isRed;
        Robot.Init(opMode);
        autonomous = true;
        var autoCommand = autoSequence.get();

        //run logic while disabled
        while (!opMode.opModeIsActive()) {
            Robot.Periodic();
        }

        //clear out auto
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        CommandScheduler.getInstance().schedule(autoCommand);
        Robot.isEnabled = true;
        RobotState.initRun();

        //run while enabled
        while (!opMode.isStopRequested()) {
            Robot.Periodic();
        }
    }

    public static boolean isEnabled() {
        return isEnabled;
    }

    @Config
    public static class RobotConfig {
        public static double CALIBRATE_SHOT_RPM = 1000;
        public static double CALIBRATE_SHOT_HOOD = 0.3;
        public static long SPINDEXER_SHOT_DELAY = 120;
        /// How long must a ball be read before we update the bay status with a color
        public static double BALL_BAY_TIME_MS = 15;
        public static double TURRET_OFFSET_DEG = -157.8;
        /// How much power do we need to start the motor from zero
        public static double SHOOTER_MOTOR_KS = 0.02;
        /// What is the power needed to get to certain RPMs
        public static double SHOOTER_MOTOR_KV = 0.00019;
        /// How much power do we add based on the amount of error we have
        public static double SHOOTER_MOTOR_KP = 0.00028;
        public static int INTAKE_SPEED = 1;
        public static int COUNTS_PER_BAY = 448;
        public static int POSITION_TOLERANCE = 30;
        public static double HELIDEXER_P = 0.7;
		public static double SHOOTER_RPM_SMOOTHER = 0.25;
        public static double CAMERA_SERVO_POS = 0.5;
        public static double TURRET_CAMERA_AIM_P = 0.005;
        public static double TURRET_KP = 0.0027;
        public static double TURRET_KI = 0;
        public static double TURRET_KD = 0;
        public static double TURRET_KS = -1;
    }

    public static Command commandFloorLoad() {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        turret.setLedCommand(GoBildaLedColors.Blue),
                        helidexer.commandFloorLoadUntilBall(1),
                        helidexer.commandFloorLoadUntilBall(2),
                        helidexer.commandFloorLoadUntilBall(3),
                        turret.setLedCommand(GoBildaLedColors.Orange)
                ),
                intake.runIntake()
        ).andThen(new ParallelCommandGroup(
                intake.runOuttake().withTimeout(1000),
                helidexer.primeForMotif()
        ));
    }

    public static Command resetRobotPose() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        shooter.setRpmWithFinished(100),
                        turret.commandTurretAngle(0)
                ),
                drivetrain.resetPoseCommand(),
                drivetrain.resetFieldOriented()
        );
    }

    public static Command calibrateShooter() {
        //drop balls in the HP hole and the robot shoots them
        return new ParallelCommandGroup(
                turret.commandTurretAngle(0),
                shooter.calibrateShot(),
                hoodAngle.calibrateShot(),
                intake.runIntake(),
                new RepeatCommand(new SequentialCommandGroup(
                        helidexer.commandFloorLoadUntilBall(1),
                        helidexer.advanceBay(),
                        helidexer.advanceBay(),
                        helidexer.shootAll()
                ))
        );
    }

    public static Command shootAllBalls(boolean motif) {
        Command primeCommand;
        if(motif) {
            primeCommand = helidexer.primeForMotif();
        } else {
            primeCommand = helidexer.primeForShot();
        }

        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        primeCommand,
                        /*new ParallelCommandGroup(
                                new WaitUntilCommand(readyToShoot()).withTimeout(2000),

                        ),*/
                        helidexer.shootAll(),
                        turret.setLedCommand(GoBildaLedColors.Off),
                        new InstantCommand(() -> {
                            CommandScheduler.getInstance().cancelAll();
                            CommandScheduler.getInstance().schedule(commandFloorLoad());
                        })
                ),
                logTimer.startLog(),
                shooter.autoShotRpm().perpetually(),
                hoodAngle.autoShotHood().perpetually(),
                turret.centerTurretViaPosition().perpetually()
                //drivetrain.holdAtSpot().perpetually()
        );
    }

    public static Command flipAlliance() {
        return new CommandBase() {
            @Override
            public void execute() {
                Robot.IsRed = !Robot.IsRed;
                setLed();
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    private static void setLed() {
        boolean ready = true;
        if (isEnabled() == false) {
            if (helidexer.isMotifPossible() == false) {
                opMode.telemetry.addLine("3 balls not loaded correctly.");
                ready = false;
            }
            if (helidexer.getBayState(1) != SpinBay.BayState.Green) {
                opMode.telemetry.addLine("Green must be loaded in bay 1 and pointed at intake.");
                ready = false;
            }
            if(Math.abs(turret.getAngle()) > 10) {
                opMode.telemetry.addLine("Turret must be pointed straight forward or zeroed.");
                ready = false;
            }
            //check position
            if(autonomous && !vision.seesMotif() ) {
                opMode.telemetry.addLine("Limelight does not see motif tag.");
                ready = false;
                /*
                //this logic won't work because we can't get an accurate vision pose with a disabled servo
                var visionPose = vision.getVisionPose();
                var drivePose = drivetrain.getPose();
                if (!vision.seesMotif() || visionPose.isEmpty()) {
                    opMode.telemetry.addLine("Limelight does not see motif and goal tag.");
                    ready = false;
                }

                if(visionPose.isPresent()) {
                    var deltaX = drivePose.getX() - visionPose.get().getX();
                    var deltaY = drivePose.getY() - visionPose.get().getY();
                    var dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                    //inches off from auto pose
                    if (dist > 4) {
                        opMode.telemetry.addLine("Robot is not in the right position.");
                        ready = false;
                    }
                }
                 */
            }
        }
        //show status
        if (ready == false) {
            allianceLed.setPosition(GoBildaLedColors.Yellow);
        } else if (readyToShoot().getAsBoolean()) {
            allianceLed.setPosition(GoBildaLedColors.Green);
        } else if (Robot.IsRed) {
            allianceLed.setPosition(GoBildaLedColors.Red);
        } else {
            allianceLed.setPosition(GoBildaLedColors.Blue);
        }
    }

    public static Command resetRobot(Pose startPose) {
        return new ParallelCommandGroup(
                drivetrain.DriveToPose(startPose),
                turret.commandTurretAngle(0),
                shooter.setRpm(0)
        );
    }

    public static Command presetShot(Pose shootingPose) {
        return new ParallelCommandGroup(
                shooter.preShotRpm(shootingPose).perpetually(),
                turret.preShotTurret(shootingPose).perpetually(),
                hoodAngle.preShotHood(shootingPose).perpetually()
        );
    }

    public static Command autoShootMotif() {
        return new SequentialCommandGroup(
                helidexer.primeForMotif(),
                helidexer.shootAll()
        );
    }
}
