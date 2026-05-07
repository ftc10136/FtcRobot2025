package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.TimerEvent;

@Autonomous
public class RedRearBalls extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.runAutonomous(this, this::getAutoSequence, true);
    }

    private Command getAutoSequence() {
        var base = new RedRearBase();
        Robot.drivetrain.setPose(base.GetStartPose());
        TimerEvent timer = new TimerEvent();
        /*timer.timerEvent(2.5).whenActive(() -> {
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().schedule(base.LeaveShotSpot());
            Robot.opMode.telemetry.addLine("Leaving now!!!");
        });*/
        return new SequentialCommandGroup(
                timer,
                base.LeaveStartAndShoot(),
                base.BackSpike(),
                new RepeatCommand(base.BackBall()).interruptOn(timer.onElapsed(28.5)),
                base.LeaveShotSpot()
        );
    }
}
