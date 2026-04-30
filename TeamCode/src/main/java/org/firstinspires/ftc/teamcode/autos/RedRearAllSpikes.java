package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class RedRearAllSpikes extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.runAutonomous(this, this::getAutoSequence, true);
    }

    private Command getAutoSequence() {
        var base = new RedRearBase();
        return new SequentialCommandGroup(
                base.LeaveShotSpot(),
                base.BackSpike(),
                base.MidSpike(),
                base.FrontSpike(),
                base.LeaveShotSpot()
        );
    }
}
