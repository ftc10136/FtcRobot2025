package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths;

@Autonomous
public class BlueBallAuto extends LinearOpMode {
    private Pose lookPose = new Pose(45, 24, Math.PI);
    private Pose shootPose = new Pose(56.7, 22.5, 3.);
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
        return new RepeatCommand(new SequentialCommandGroup(
                Robot.drivetrain.DriveToPose(shootPose, true),
                Robot.shootMotif(),
                new ParallelDeadlineGroup(
                        Robot.commandFloorLoad(),
                        new RepeatCommand(new SequentialCommandGroup(
                            Robot.drivetrain.DriveToPose(lookPose, false).interruptOn(Robot.vision.seesBalls()),
                            Robot.drivetrain.driveToBall()
                        ))
                ).withTimeout(10000)
        ));
    }
}
