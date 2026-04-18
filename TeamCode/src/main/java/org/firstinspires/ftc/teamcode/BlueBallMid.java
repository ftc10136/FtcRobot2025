package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

@Autonomous
public class BlueBallMid extends LinearOpMode {
    //MODIFICATIONS: Path2 add .setNoDeceleration()
    /// START AUTO GENERATED CODE ------------------------------------------------------------------

    public static class Paths {
        public PathChain Start;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Start = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.138, 119.517),

                                    new Pose(59.724, 77.379)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(218))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.724, 77.379),

                                    new Pose(40.759, 60.379)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(218), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.759, 60.379),

                                    new Pose(17.172, 59.483)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.172, 59.483),
                                    new Pose(26.328, 66.379),
                                    new Pose(15.876, 66.862)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.876, 66.862),
                                    new Pose(44.466, 70.483),
                                    new Pose(59.724, 77.552)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }
    /// END AUTO GENERATED CODE --------------------------------------------------------------------

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
        var startPose = paths.Start.getPose(new PathChain.PathT(0,0));
        Robot.drivetrain.setPose(startPose);

        return new SequentialCommandGroup(
                //drive to shoot position
                Robot.drivetrain.followPath(paths.Start,false,1),
                Robot.drivetrain.followPath(paths.Path2,false,1),
                Robot.drivetrain.followPath(paths.Path3,false,0.2),
                Robot.drivetrain.followPath(paths.Path4,false,1),
                Robot.drivetrain.followPath(paths.Path5,false,1),

                //This is a hack that resets the robot back to starting position
                Robot.drivetrain.DriveToPose(startPose)
                /*
                Robot.shootMotif(),
                new ParallelDeadlineGroup(
                        Robot.commandFloorLoad(),
                        new RepeatCommand(new SequentialCommandGroup(
                            Robot.drivetrain.DriveToPose(lookPose, false).interruptOn(Robot.vision.seesBalls()),
                            Robot.drivetrain.driveToBall()
                        ))
                ).withTimeout(10000)*/
        );
    }
}
