package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.Ballevator;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.HoodAngle;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Robot {
    public static OpMode opMode;
    public static Drivetrain drivetrain;
    public static Controls controls;
    public static Spindexer spindexer;
    public static Intake intake;
    public static Ballevator ballevator;
    public static Turret turret;
    public static Shooter shooter;
    public static HoodAngle hoodAngle;
    public static Vision vision;

    public static void Init(OpMode inMode) {
        opMode = inMode;
        controls = new Controls();
        drivetrain = new Drivetrain();
        spindexer = new Spindexer();
        intake = new Intake();
        ballevator = new Ballevator();
        turret = new Turret();
        shooter = new Shooter();
        hoodAngle = new HoodAngle();
        vision = new Vision();

        FtcDashboard.getInstance().setTelemetryTransmissionInterval(20);
    }

    public static void Periodic() {
        CommandScheduler.getInstance().run();
        opMode.telemetry.update();
    }

    public static void logPacket(TelemetryPacket packet) {
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static void scheduleTeleop() {
        //default commands that run when the robot is idle
        drivetrain.setDefaultCommand(new Drivetrain.TeleopDrive());

        //button commands
        controls.hpLoadActive().whileActiveContinuous(commandHumanLoad());
        controls.floorLoadActive().whileActiveContinuous(commandFloorLoad());

        //test commands
        //new Trigger(()->Robot.opMode.gamepad1.start).whileActiveContinuous(turret.testTurret());
        Trigger shootCalibration = new Trigger(()->Robot.opMode.gamepad1.back);
        shootCalibration.toggleWhenActive(calibrateShooter());
    }

    @Config
    public static class RobotConfig {
        public static double SPINDEXER_OFFSET = 0.591;
        public static double CALIBRATE_SHOT_RPM = 1000;
        public static double CALIBRATE_SHOT_HOOD = 0.3;
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
        return new ParallelCommandGroup(
                intake.runIntake(),
                new SequentialCommandGroup(
                        ballevator.commandDown(),
                        spindexer.commandFloorLoadUntilBall(1),
                        spindexer.commandFloorLoadUntilBall(2),
                        spindexer.commandFloorLoadUntilBall(3)
                ));
    }

    public static Command calibrateShooter() {
        return new ParallelCommandGroup(
                //intake.runIntake(),
                turret.centerTurretViaVision().perpetually(),
                shooter.calibrateShot(),
                hoodAngle.calibrateShot(),
                new RepeatCommand(new SequentialCommandGroup(
                        ballevator.commandDown(),
                        spindexer.commandHpLoadUntilBall(1),
                        spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.Shoot),
                        ballevator.commandUp(),
                        new WaitCommand(300)
                ))
        );
    }
}
