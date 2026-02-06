package org.firstinspires.ftc.teamcode;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths;

@Autonomous
public class RedTriangleTest extends LinearOpMode {
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
        Robot.drivetrain.setPose(paths.Path2.getPose(new PathChain.PathT(0,0)));
        return new SequentialCommandGroup(
                //Robot.drivetrain.followPath(paths.Path1),
                //new WaitCommand(5000),
                Robot.drivetrain.followPath(paths.Path2),
                new WaitCommand(5000),
                Robot.drivetrain.followPath(paths.Path3)
        );
    }
}
