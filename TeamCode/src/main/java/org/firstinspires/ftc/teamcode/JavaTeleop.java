package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

@TeleOp
@Disabled
public class JavaTeleop extends OpMode {
    @Override
    public void init() {
        Robot.Init(this);
    }

    @Override
    public void init_loop() {
        Robot.Periodic();
    }

    @Override
    public void start() {
        //reset all commands that were running
        CommandScheduler.getInstance().cancelAll();
        Robot.scheduleTeleop();
    }

    @Override
    public void loop() {
        Robot.Periodic();
    }
}
