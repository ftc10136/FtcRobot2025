package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.LoggerCommandTimer;

@Autonomous
public class ATest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.runAutonomous(this, this::getAutoSequence, false);
    }

    private Command getAutoSequence() {
        var logTimer = new LoggerCommandTimer("Auto Times");
        var base = new BlueRearBase();
        Robot.drivetrain.setPose(base.GetStartPose());
        return new SequentialCommandGroup(
                logTimer.startLog(),
                base.LeaveStartAndShoot(),
                logTimer.addEntry("LeaveStartAndShoot"),
                base.FrontSpike(),
                logTimer.addEntry("FrontSpike"),
                base.BackBall(),
                logTimer.addEntry("BackBall"),
                base.LeaveShotSpot(),
                logTimer.finishLog("LeaveShotSpot")
        );
    }
}
