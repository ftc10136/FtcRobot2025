package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.Ballevator;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainPP;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.HoodAngle;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.livoniawarriors.GoBildaLedColors;
import org.livoniawarriors.LoggerCommandTimer;
import org.livoniawarriors.SequentialCommandGroup;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class Robot {
    public static OpMode opMode;
    public static DrivetrainPP drivetrain;
    public static Controls controls;
    public static Spindexer spindexer;
    public static Intake intake;
    public static Ballevator ballevator;
    public static Turret turret;
    public static Shooter shooter;
    public static HoodAngle hoodAngle;
    public static Vision vision;
    private static Servo allianceLed;

    public static boolean IsRed = false;
    public static RobotTypeEnum RobotType = RobotTypeEnum.Competition;
    private static TelemetryPacket packet;

    private static LoggerCommandTimer logTimer;
    private static boolean isEnabled;

    public enum RobotTypeEnum {
        Competition,
        Programming
    }

    public static void Init(OpMode inMode) {
        opMode = inMode;
        isEnabled = false;
        CommandScheduler.getInstance().setBulkReading(opMode.hardwareMap, LynxModule.BulkCachingMode.AUTO);
        packet = new TelemetryPacket();
        var pose = new Pose(72,72,Math.PI/2, PedroCoordinates.INSTANCE);
        //if we have already run, clear out the old running subsystems
        if (drivetrain != null) {
            pose = drivetrain.getPose();
            CommandScheduler.getInstance().unregisterSubsystem(drivetrain, spindexer, intake,
                    ballevator, turret, shooter, hoodAngle, vision);
        }
        controls = new Controls();
        drivetrain = new DrivetrainPP();
        spindexer = new Spindexer();
        intake = new Intake();
        ballevator = new Ballevator();
        turret = new Turret();
        shooter = new Shooter();
        hoodAngle = new HoodAngle();
        vision = new Vision();
        allianceLed = opMode.hardwareMap.get(Servo.class, "RGB-Alliance");
        setLed();
        drivetrain.setPose(pose);

        //we ran before, clear out all the old stuff running in the schedule
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(20);

        //buttons that run while disabled
        controls.flipAlliance().whenActive(flipAlliance());
        controls.resetTurretAngle().whenActive(turret.resetZero());

        logTimer = new LoggerCommandTimer("Shot Times");
        //uncomment this line to do timing analysis on commands.  This will cause them to run twice.
        //CommandScheduler.getInstance().onCommandExecute(Robot::logCommandTiming);
    }

    public static void Periodic() {
        long startTime = System.nanoTime();
        loggingTime = 0;
        CommandScheduler.getInstance().run();
        //periodicTiming();  //note, this causes all the periodic functions to run twice!

        double deltaTime = (System.nanoTime() - startTime) / 1_000_000.;
        packet.put("TimingMs/LoggingTime", loggingTime);
        packet.put("TimingMs/CommandTime", deltaTime);
        logPacket(packet);

        opMode.telemetry.addData("TurretAngle", turret.getAngle());
        opMode.telemetry.addData("ShotDistance", drivetrain.getGoalDistance());
        opMode.telemetry.addData("Command TimeMs", deltaTime);
        opMode.telemetry.addData("_RPM Ready", shooter.atTarget());
        opMode.telemetry.addData("_Turret Ready", turret.atTarget());
        opMode.telemetry.addData("_Hood Ready", hoodAngle.atTarget());
        opMode.telemetry.update();
    }

    private static void logCommandTiming(Command command) {
        long startTime2 = System.nanoTime();
        command.execute();
        double deltaTime = (System.nanoTime() - startTime2) / 1_000_000.;
        packet.put("TimingMs/" + command, deltaTime);
    }

    private void periodicTiming() {
        try {
            Class<?> clazz = CommandScheduler.class;

            // 2. Get the specific private field
            // Use getDeclaredField() to access private fields declared within the class
            // getFields() only returns public fields
            Field privateStringField = clazz.getDeclaredField("m_subsystems");

            // 3. Make the private field accessible
            // This is necessary to bypass Java language access control checks
            privateStringField.setAccessible(true);

            // 4. Get the value of the field from the specific object instance
            @SuppressWarnings("unchecked")
            Map<Subsystem, Command> fieldValue = (Map<Subsystem, Command>) privateStringField.get(CommandScheduler.getInstance());
            assert fieldValue != null;
            for (Subsystem subsystem : fieldValue.keySet()) {
                long startTime2 = System.nanoTime();
                subsystem.periodic();
                double deltaTime = (System.nanoTime() - startTime2) / 1_000_000.;
                packet.put("TimingMs/"+subsystem.toString(), deltaTime);
            }

        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
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

        //default commands that run when the robot is idle
        drivetrain.startTeleopDrive(true);
        drivetrain.setDefaultCommand(drivetrain.teleopDrive());
        //turret.setDefaultCommand(turret.centerTurretViaPosition().perpetually());
        shooter.setDefaultCommand(shooter.autoShotRpm().perpetually());
        hoodAngle.setDefaultCommand(hoodAngle.autoShotHood().perpetually());

        //button commands
        controls.hpLoadActive().whileActiveContinuous(commandHumanLoad());
        controls.floorLoadActive().toggleWhenActive(commandFloorLoad());
        controls.resetFieldOriented().whenActive(drivetrain.resetFieldOriented());
        controls.shootActive().toggleWhenActive(shootAllBalls());
        controls.bumpSpindexerLeft().whileActiveContinuous(spindexer.bumpSpindexer(true));
        controls.bumpSpindexerRight().whileActiveContinuous(spindexer.bumpSpindexer(false));
        controls.shootMotif().whileActiveContinuous(shootMotif());

        controls.flipAlliance().whenActive(flipAlliance());
        controls.resetTurretAngle().whenActive(turret.resetZero());
        controls.bumpTurretRotationsLeft().whenActive(turret.bumpRotations(-1));
        controls.bumpTurretRotationsRight().whenActive(turret.bumpRotations(1));
        controls.resetBays().whenActive(spindexer.resetBays());

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

    public static boolean isEnabled() {
        return isEnabled;
    }

    @Config
    public static class RobotConfig {
        //read position feedback from the dashboard and this should match
        public static double SPINDEXER_OFFSET_COMP = 0.364;
        public static double CALIBRATE_SHOT_RPM = 1000;
        public static double CALIBRATE_SHOT_HOOD = 0.3;
        public static long SPINDEXER_SHOT_DELAY = 120;
        public static long BALLEVATOR_UP_TIMEOUT = 300;
        /// How long must a ball be read before we update the bay status with a ball
        public static double BALL_BAY_TIME_MS = 1;
        public static double TURRET_OFFSET_DEG = 26.3;
        /// How much power do we need to start the motor from zero
        public static double SHOOTER_MOTOR_KS = 0.02;
        /// What is the power needed to get to certain RPMs
        public static double SHOOTER_MOTOR_KV = 0.00019;
        /// How much power do we add based on the amount of error we have
        public static double SHOOTER_MOTOR_KP = 0.00028;
        ///the higher this value, the more of current readings we use
        public static double SHOOTER_RPM_SMOOTHER = 0.25;

        public static double SPINDEXER_OFFSET_PROG = 0.587;
    }

    public static Command commandHumanLoad() {
        return new SequentialCommandGroup(
                ballevator.commandDown(),
                spindexer.commandHpLoadUntilBall(1),
                spindexer.commandHpLoadUntilBall(2),
                spindexer.commandHpLoadUntilBall(3)
        );
    }

    public static Command commandFloorLoad() {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        turret.setLedCommand(GoBildaLedColors.Blue),
                        ballevator.commandDown(),
                        spindexer.commandFloorLoadUntilBall(1),
                        spindexer.commandFloorLoadUntilBall(2),
                        spindexer.commandFloorLoadUntilBall(3),
                        turret.setLedCommand(GoBildaLedColors.Orange)
                ),
                intake.runIntake()
        ).andThen(new ParallelCommandGroup(
                intake.runOuttake().withTimeout(1000),
                spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.Shoot)
        ));
    }

    public static Command calibrateShooter() {
        //drop balls in the HP hole and the robot shoots them
        return new ParallelCommandGroup(
                turret.commandTurretAngle(0),
                shooter.calibrateShot(),
                hoodAngle.calibrateShot(),
                intake.runIntake(),
                new RepeatCommand(new SequentialCommandGroup(
                        ballevator.commandDown(),
                        spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.FloorIntake),
                        spindexer.commandFloorLoadUntilBall(1),
                        spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.Shoot),
                        ballevator.commandUp(),
                        new WaitCommand(300),
                        spindexer.clearCurrentBay()
                ))
        );
    }

    public static Command shootBall(int bay) {
        var fastTimer = new LoggerCommandTimer("FastBall" + bay);
        return new SequentialCommandGroup(
                fastTimer.startLog(),
                ballevator.commandDown().alongWith(fastTimer.addEntry(bay + "BallStart")),
                spindexer.commandSpindexerPos(bay, Spindexer.SpindexerType.Shoot).alongWith(fastTimer.addEntry(bay + "BallElevatorDown")),
                new WaitCommand(RobotConfig.SPINDEXER_SHOT_DELAY).alongWith(fastTimer.addEntry(bay + "BallSpindexer")),
                ballevator.commandUp().withTimeout(RobotConfig.BALLEVATOR_UP_TIMEOUT).alongWith(fastTimer.addEntry(bay + "BallSleep")),
                spindexer.clearBayState(bay).alongWith(fastTimer.addEntry(bay + "BallElevatorUp")),
                fastTimer.finishLog(bay + "BallCleared")

        );
    }

    public static Command fastShot(int bay) {
        return new FastSequentialCommand(bay);
    }

    public static Command shootAllBalls() {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        new WaitUntilCommand(readyToShoot()).withTimeout(2000),
                        logTimer.addEntry("StartShot"),
                        new ConditionalCommand(fastShot(1), new InstantCommand(), spindexer.hasBall(1)),
                        logTimer.addEntry("Ball1Shot"),
                        new ConditionalCommand(fastShot(2), new InstantCommand(), spindexer.hasBall(2)),
                        logTimer.addEntry("Ball2Shot"),
                        new ConditionalCommand(fastShot(3), new InstantCommand(), spindexer.hasBall(3)),
                        logTimer.finishLog("Ball3Shot"),
                        turret.setLedCommand(GoBildaLedColors.Off),
                        ballevator.commandDown(),
                        spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.FloorIntake),
                        new InstantCommand(() -> {
                            CommandScheduler.getInstance().cancelAll();
                            CommandScheduler.getInstance().schedule(commandFloorLoad());
                        })
                ),
                logTimer.startLog(),
                shooter.autoShotRpm().perpetually(),
                hoodAngle.autoShotHood().perpetually(),
                turret.centerTurretViaPosition().perpetually(),
                drivetrain.holdAtSpot().perpetually()
        );
    }

    public static Command shootMotif() {
        var map = new HashMap<Object, Command>();
        map.put(Vision.Motifs.GPP, shootMotif(Spindexer.BayState.Green, Spindexer.BayState.Purple, Spindexer.BayState.Purple));
        map.put(Vision.Motifs.PGP, shootMotif(Spindexer.BayState.Purple, Spindexer.BayState.Green, Spindexer.BayState.Purple));
        map.put(Vision.Motifs.PPG, shootMotif(Spindexer.BayState.Purple, Spindexer.BayState.Purple, Spindexer.BayState.Green));
        return new SelectCommand(map,() -> Robot.vision.getSeenMotif());
    }

    public static Command shootMotif(Spindexer.BayState color1, Spindexer.BayState color2, Spindexer.BayState color3) {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        //this is needed to run the spindexer initially to "wake up" the servo
                        ballevator.commandDown(),
                        spindexer.commandSpindexerPos(Spindexer.getOffset() + 0.2825).withTimeout(100),
                        spindexer.commandSpindexerPos(Spindexer.getOffset() + 0.2525).withTimeout(100),
                        spindexer.commandSpindexerColor(color1),
                        new WaitCommand(RobotConfig.SPINDEXER_SHOT_DELAY),
                        new WaitUntilCommand(readyToShoot()).withTimeout(2000),
                        ballevator.commandUp().withTimeout(RobotConfig.BALLEVATOR_UP_TIMEOUT),
                        spindexer.clearCurrentBay(),
                        ballevator.commandDown(),
                        spindexer.commandSpindexerColor(color2),
                        new WaitCommand(RobotConfig.SPINDEXER_SHOT_DELAY),
                        ballevator.commandUp().withTimeout(RobotConfig.BALLEVATOR_UP_TIMEOUT),
                        spindexer.clearCurrentBay(),
                        ballevator.commandDown(),
                        spindexer.commandSpindexerColor(color3),
                        new WaitCommand(RobotConfig.SPINDEXER_SHOT_DELAY),
                        ballevator.commandUp().withTimeout(RobotConfig.BALLEVATOR_UP_TIMEOUT),
                        spindexer.clearCurrentBay(),
                        ballevator.commandDown(),
                        turret.setLedCommand(GoBildaLedColors.Off),
                        spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.FloorIntake)
                ),
                shooter.autoShotRpm().perpetually(),
                hoodAngle.autoShotHood().perpetually(),
                turret.centerTurretViaPosition()
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
        if (Robot.IsRed) {
            allianceLed.setPosition(GoBildaLedColors.Red);
        } else {
            allianceLed.setPosition(GoBildaLedColors.Blue);
        }
    }

    private static class FastSequentialCommand extends CommandBase {

        private final int bay;
        private LoggerCommandTimer fastTimer;

        public FastSequentialCommand(int bay) {
            this.bay = bay;
        }

        @Override
        public void initialize() {
            fastTimer = new LoggerCommandTimer("FastBall" + bay);
            var allHubs = opMode.hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            }
        }

        @Override
        public void execute() {
            var sequence = List.of(
                    fastTimer.startLog(),
                    turret.stopTurret(),
                    drivetrain.holdAtSpot(),
                    fastTimer.addEntry(bay + "BallStart"),
                    ballevator.commandDown(),
                    fastTimer.addEntry(bay + "BallElevatorDown"),
                    spindexer.commandSpindexerPos(bay, Spindexer.SpindexerType.Shoot),
                    fastTimer.addEntry(bay + "BallSpindexer"),
                    new WaitCommand(RobotConfig.SPINDEXER_SHOT_DELAY),
                    fastTimer.addEntry(bay + "BallSleep"),
                    ballevator.commandUp().withTimeout(RobotConfig.BALLEVATOR_UP_TIMEOUT),
                    fastTimer.addEntry(bay + "BallElevatorUp"),
                    spindexer.clearBayState(bay),
                    fastTimer.finishLog(bay + "BallCleared")
            );
            for(var command : sequence) {
                runCommand(command);
            }
        }

        @Override
        public void end(boolean interrupted) {
            CommandScheduler.getInstance().setBulkReading(opMode.hardwareMap, LynxModule.BulkCachingMode.AUTO);
        }

        @Override
        public boolean isFinished() {
            return true;
        }

        private void runCommand(Command command) {
            Timing.Timer timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
            command.initialize();
            timer.start();
            do {
                command.execute();
            } while(!command.isFinished() && !timer.done());
            command.end(timer.done());
        }
    }
}
