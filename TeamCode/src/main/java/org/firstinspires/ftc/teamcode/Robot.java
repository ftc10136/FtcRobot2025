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

import org.firstinspires.ftc.teamcode.subsystems.Ballevator;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainPP;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.HoodAngle;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.livoniawarriors.GoBildaLedColors;
import org.livoniawarriors.SequentialCommandGroup;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
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
    public static int stepNum = 0;
    private static TelemetryPacket packet;

    public enum RobotTypeEnum {
        Competition,
        Programming
    }

    public static void Init(OpMode inMode) {
        opMode = inMode;
        CommandScheduler.getInstance().setBulkReading(opMode.hardwareMap, LynxModule.BulkCachingMode.MANUAL);
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
    }

    public static void Periodic() {
        long startTime = System.nanoTime();
        loggingTime = 0;
        CommandScheduler.getInstance().run();
        //periodicTiming();  //note, this causes all the periodic functions to run twice!

        double deltaTime = (System.nanoTime() - startTime) / 1_000_000.;
        packet.put("TimingMs/LoggingTime", loggingTime);
        logPacket(packet);

        opMode.telemetry.addData("Step", stepNum);
        opMode.telemetry.addData("TurretAngle", turret.getAngle());
        opMode.telemetry.addData("Command TimeMs", deltaTime);
        opMode.telemetry.addData("_RPM Ready", shooter.atTarget());
        opMode.telemetry.addData("_Turret Ready", turret.atTarget());
        opMode.telemetry.addData("_Hood Ready", hoodAngle.atTarget());
        opMode.telemetry.update();
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
        //shootCalibration.toggleWhenActive(calibrateShooter());
        shootCalibration.whileActiveContinuous(new ParallelDeadlineGroup(
                commandFloorLoad(),
                drivetrain.driveToBall()
        ));
    }

    public static void setAutonomous(Command autoSequence) {
        //clear out auto
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        CommandScheduler.getInstance().schedule(autoSequence);
    }

    @Config
    public static class RobotConfig {
        public static double SPINDEXER_OFFSET_COMP = 0.375;
        public static double SPINDEXER_OFFSET_PROG = 0.587;
        public static double CALIBRATE_SHOT_RPM = 1000;
        public static double CALIBRATE_SHOT_HOOD = 0.3;
        public static long SPINDEXER_SHOT_DELAY = 0;
        public static long BALLEVATOR_UP_TIMEOUT = 300;
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
                        turret.setLedCommand(GoBildaLedColors.Orange),
                        spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.Shoot)
                ),
                intake.runIntake());
    }

    public static Command calibrateShooter() {
        //drop balls in the HP hole and the robot shoots them
        return new ParallelCommandGroup(
                turret.commandTurretAngle(0),
                shooter.calibrateShot(),
                hoodAngle.calibrateShot(),
                new RepeatCommand(new SequentialCommandGroup(
                        ballevator.commandDown(),
                        spindexer.commandHpLoadUntilBall(1),
                        spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.Shoot),
                        ballevator.commandUp(),
                        new WaitCommand(300),
                        spindexer.clearCurrentBay()
                ))
        );
    }

    public static Command shootBall(int bay) {
        return new SequentialCommandGroup(
                logStep(10),
                ballevator.commandDown(),
                logStep(11),
                spindexer.commandSpindexerPos(bay, Spindexer.SpindexerType.Shoot),
                logStep(12),
                new WaitCommand(RobotConfig.SPINDEXER_SHOT_DELAY),
                logStep(13),
                ballevator.commandUp().withTimeout(RobotConfig.BALLEVATOR_UP_TIMEOUT),
                logStep(14),
                spindexer.clearBayState(bay),
                logStep(15)
        );
    }

    public static Command shootAllBalls() {
        return new ParallelCommandGroup(
                shooter.autoShotRpm().perpetually(),
                hoodAngle.autoShotHood().perpetually(),
                turret.centerTurretViaPosition().perpetually(),
                new SequentialCommandGroup(
                        logStep(1),
                        new WaitUntilCommand(readyToShoot()).withTimeout(2000),
                        logStep(2),
                        new ConditionalCommand(shootBall(1), new InstantCommand(), spindexer.hasBall(1)),
                        logStep(3),
                        new ConditionalCommand(shootBall(2), new InstantCommand(), spindexer.hasBall(2)),
                        new ConditionalCommand(shootBall(3), new InstantCommand(), spindexer.hasBall(3)),
                        turret.setLedCommand(GoBildaLedColors.Off),
                        ballevator.commandDown(),
                        spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.FloorIntake),
                        new InstantCommand(() -> CommandScheduler.getInstance().schedule(commandFloorLoad()))
                )
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


    public static Command logStep(int step) {
        return new CommandBase() {
            @Override
            public void execute() {
                stepNum = step;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }
}
