package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Spindexer.Spindexer;

public class Robot {
    public static OpMode opMode;
    public static Drivetrain drivetrain;
    public static Controls controls;
    public static Spindexer spindexer;

    public static void Init(OpMode inMode) {
        opMode = inMode;
        controls = new Controls();
        drivetrain = new Drivetrain();
        spindexer = new Spindexer();

        drivetrain.setDefaultCommand(new Drivetrain.TeleopDrive());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(20);
    }

    public static void Periodic() {
        CommandScheduler.getInstance().run();
    }

    public static void scheduleTeleop() {
        controls.hpLoadActive().whileActiveContinuous(commandHumanLoad());
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
}


