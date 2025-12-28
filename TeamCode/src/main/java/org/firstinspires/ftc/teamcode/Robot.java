package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

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
    }

    public static void Periodic() {
        CommandScheduler.getInstance().run();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
