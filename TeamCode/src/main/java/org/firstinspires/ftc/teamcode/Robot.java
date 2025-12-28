package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Spindexer.Spindexer;

public class Robot {
    public static OpMode opMode;
    public static Drivetrain drivetrain;
    public static Controls controls;
    public static Spindexer spindexer;
    public static Intake intake;

    public static void Init(OpMode inMode) {
        opMode = inMode;
        controls = new Controls();
        drivetrain = new Drivetrain();
        spindexer = new Spindexer();
        intake = new Intake();

        FtcDashboard.getInstance().setTelemetryTransmissionInterval(20);
    }

    public static void Periodic() {
        CommandScheduler.getInstance().run();
    }

    public static void scheduleTeleop() {
        //default commands that run when the robot is idle
        drivetrain.setDefaultCommand(new Drivetrain.TeleopDrive());

        //button commands
        controls.hpLoadActive().whileActiveContinuous(commandHumanLoad());
        controls.floorLoadActive().whileActiveContinuous(commandFloorLoad());
    }

    @Config
    public static class RobotConfig {
        public static double SPINDEXER_OFFSET = 0.591;
    }

    public static Command commandHumanLoad() {
        return new SequentialCommandGroup(
                spindexer.commandHpLoadUntilBall(1),
                spindexer.commandHpLoadUntilBall(2),
                spindexer.commandHpLoadUntilBall(3)
        );
    }

    public static Command commandFloorLoad() {
        return new ParallelCommandGroup(
                intake.runIntake(),
                new SequentialCommandGroup(
                        spindexer.commandFloorLoadUntilBall(1),
                        spindexer.commandFloorLoadUntilBall(2),
                        spindexer.commandFloorLoadUntilBall(3)
                ));
    }
}


