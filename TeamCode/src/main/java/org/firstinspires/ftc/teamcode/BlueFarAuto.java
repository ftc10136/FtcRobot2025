package org.firstinspires.ftc.teamcode;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths;

@Autonomous
public class BlueFarAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.Init(this);
        var autoCommand = getAutoSequence();

        //run logic while disabled
        while (!opModeIsActive()) {
            Robot.Periodic();
        }

        //reset all commands that were running
        Robot.setAutonomous(autoCommand);

        //run while enabled
        while (!isStopRequested()) {
            Robot.Periodic();
        }
    }

    private Command getAutoSequence() {
        Paths paths = new Paths(Robot.drivetrain.getFollower());
        Robot.drivetrain.setPose(paths.BlueBackBalls.getPose(new PathChain.PathT(0,0)));
        return new SequentialCommandGroup(
                Robot.shootMotif(),
                Robot.drivetrain.followPath(paths.BlueBackBalls, false, 1),
                new ParallelDeadlineGroup(
                        Robot.commandFloorLoad(),
                        Robot.drivetrain.followPath(paths.BlueBackIntake, true, 0.3)
                ).withTimeout(10000),
                Robot.drivetrain.followPath(paths.BlueBackToShoot, true, 1),
                Robot.shootMotif()
        );
    }
}
