package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

@Autonomous
public class BlueFrontRamp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.runAutonomous(this, this::getAutoSequence, false);
    }

    private Command getAutoSequence() {
        Paths paths = new Paths(Robot.drivetrain.getFollower());
        var startPose = paths.Start.getPose(new PathChain.PathT(0, 0));
        var shotPose = paths.Start.endPose();
        var shot2Pose = paths.Path5.endPose();
        var shot3Pose = paths.Path7.endPose();
        var shot4Pose = paths.Path10.endPose();
        Robot.drivetrain.setPose(startPose);

        return new SequentialCommandGroup(
                //shot 1
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        //drive to shoot position
                                        Robot.drivetrain.followPath(paths.Start, false, 1)
                                        //,Robot.spindexer.commandSpindexerPos(1, Spindexer.SpindexerType.Shoot)
                                ),

                                //shoot balls
                                Robot.autoShootMotif()
                        ),
                        Robot.presetShot(shotPose)
                ),
                //shot 2 and gate
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                //get middle preset balls
                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                Robot.drivetrain.followPath(paths.Path2, false, 1),
                                                Robot.drivetrain.followPath(paths.Path3, false, 0.24)
                                        ),
                                        Robot.commandFloorLoad()
                                ),

                                //open gate
                                Robot.drivetrain.followPath(paths.Path4, false, 1),

                                //return to shoot position
                                Robot.drivetrain.followPath(paths.Path5, false, 1),

                                //shoot balls
                                Robot.autoShootMotif()
                        ),
                        Robot.presetShot(shot2Pose)
                ),
                //ramp
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                //go to ramp
                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                Robot.drivetrain.followPath(paths.Path6, false, 1),
                                                new WaitCommand(5000)
                                        ),
                                        Robot.commandFloorLoad()
                                ),

                                //return to shoot position
                                Robot.drivetrain.followPath(paths.Path7, false, 1),

                                //shoot balls
                                Robot.autoShootMotif()
                        ),
                        Robot.presetShot(shot3Pose)
                ),
                //front spike
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                //get middle preset balls
                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                Robot.drivetrain.followPath(paths.Path8, false, 1),
                                                Robot.drivetrain.followPath(paths.Path9, false, 0.24)
                                        ),
                                        Robot.commandFloorLoad()
                                ),
                                //return to shoot position
                                Robot.drivetrain.followPath(paths.Path10, false, 1),

                                //shoot balls
                                Robot.autoShootMotif()
                        ),
                        Robot.presetShot(shot4Pose)
                )
                //This is a hack that resets the robot back to starting position
                ,Robot.resetRobot(startPose)
        );
    }

    /// START AUTO GENERATED CODE ------------------------------------------------------------------
    public static class Paths {
        public PathChain Start;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Start = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.138, 119.517),

                                    new Pose(59.700, 77.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(218))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),

                                    new Pose(40.759, 59.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(218), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.759, 59.500),

                                    new Pose(17.200, 59.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.200, 59.500),
                                    new Pose(26.328, 66.379),
                                    new Pose(15.876, 66.862)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.876, 66.862),
                                    new Pose(44.466, 70.483),
                                    new Pose(59.700, 77.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),

                                    new Pose(13.500, 60.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.500, 60.500),

                                    new Pose(59.700, 77.400)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(148))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),

                                    new Pose(40.750, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.750, 84.000),

                                    new Pose(17.200, 84.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.200, 84.000),

                                    new Pose(58.700, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();
        }
    }
    /// END AUTO GENERATED CODE --------------------------------------------------------------------
}
