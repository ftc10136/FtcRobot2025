package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Drivetrain.Drivetrain;

public class Robot {
    public static OpMode opMode;
    public static Drivetrain drivetrain;

    public static void Init(OpMode inMode) {
        opMode = inMode;
        drivetrain = new Drivetrain();
    }

    public static void Periodic() {
        drivetrain.FieldCentricDriving();

        FtcDashboard.getInstance().getTelemetry().update();
    }
}
