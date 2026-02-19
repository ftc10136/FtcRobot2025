package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
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

import java.util.HashMap;
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

    public static void Init(OpMode inMode) {
        opMode = inMode;
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
    }

    public static void Periodic() {
        CommandScheduler.getInstance().run();
        opMode.telemetry.update();
    }

    public static void logPacket(TelemetryPacket packet) {
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static BooleanSupplier readyToShoot() {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                boolean status = Robot.turret.atTarget() && Robot.shooter.atTarget() && Robot.hoodAngle.atTarget();
                if(status == true) {
                    status = true;
                }
                return status;
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
        turret.setDefaultCommand(turret.centerTurretViaPosition());
        shooter.setDefaultCommand(shooter.autoShotRpm().perpetually());
        hoodAngle.setDefaultCommand(hoodAngle.autoShotHood().perpetually());

        //button commands
        controls.hpLoadActive().whileActiveContinuous(commandHumanLoad());
        controls.floorLoadActive().toggleWhenActive(commandFloorLoad());
        controls.resetFieldOriented().whenActive(drivetrain.resetFieldOriented());
        controls.shootActive().toggleWhenActive(shootAllBalls());
        controls.flipAlliance().whenActive(flipAlliance());
        controls.bumpSpindexerLeft().whileActiveContinuous(spindexer.bumpSpindexer(true));
        controls.bumpSpindexerRight().whileActiveContinuous(spindexer.bumpSpindexer(false));
        controls.shootMotif().whileActiveContinuous(shootMotif());

        //test commands
        //new Trigger(()->Robot.opMode.gamepad1.start).whileActiveContinuous(turret.testTurret());
        Trigger shootCalibration = new Trigger(() -> Robot.opMode.gamepad1.start);
        shootCalibration.toggleWhenActive(calibrateShooter());
    }

    public static void setAutonomous(Command autoSequence) {
        //clear out auto
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        CommandScheduler.getInstance().schedule(autoSequence);
    }

    @Config
    public static class RobotConfig {
        public static double SPINDEXER_OFFSET = 0.587;
        public static double CALIBRATE_SHOT_RPM = 1000;
        public static double CALIBRATE_SHOT_HOOD = 0.3;
        public static long SPINDEXER_SHOT_DELAY = 80;
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
                ballevator.commandDown(),
                spindexer.commandSpindexerPos(bay, Spindexer.SpindexerType.Shoot),
                new WaitCommand(RobotConfig.SPINDEXER_SHOT_DELAY),
                ballevator.commandUp().withTimeout(RobotConfig.BALLEVATOR_UP_TIMEOUT),
                spindexer.clearBayState(bay)
        );
    }

    public static Command shootAllBalls() {
        return new ParallelCommandGroup(
                shooter.autoShotRpm(),
                hoodAngle.autoShotHood(),
                turret.centerTurretViaVision(),
                new SequentialCommandGroup(
                        new WaitUntilCommand(readyToShoot()).withTimeout(2000),
                        new ConditionalCommand(shootBall(1), new InstantCommand(), spindexer.hasBall(1)),
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
                        spindexer.commandSpindexerPos(Robot.RobotConfig.SPINDEXER_OFFSET + 0.2825).withTimeout(100),
                        spindexer.commandSpindexerPos(Robot.RobotConfig.SPINDEXER_OFFSET + 0.2525).withTimeout(100),
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
                new SequentialCommandGroup(
                        turret.centerTurretViaPosition(),
                        turret.centerTurretViaVision()
                )
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
}
