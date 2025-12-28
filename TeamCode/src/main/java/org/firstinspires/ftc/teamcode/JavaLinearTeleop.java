package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

@TeleOp
public class JavaLinearTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.Init(this);

        //run logic while disabled
        while (!opModeIsActive()) {
            Robot.Periodic();
        }

        //reset all commands that were running
        CommandScheduler.getInstance().cancelAll();
        Robot.scheduleTeleop();

        //run while enabled
        while (opModeIsActive()) {
            Robot.Periodic();
        }
    }
}
