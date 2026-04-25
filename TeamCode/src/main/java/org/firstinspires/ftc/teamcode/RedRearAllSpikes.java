package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.TemporalCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

@Autonomous
public class RedRearAllSpikes extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.runAutonomous(this, this::getAutoSequence, true);
    }

    private Command getAutoSequence() {
        var follower = Robot.drivetrain.getFollower();
        Paths paths = new Paths(follower);
        paths.BackSpike.setCallbacks(new TemporalCallback(1,0,()->{follower.setMaxPowerScaling(0.45);}));
        paths.BackSpike.setCallbacks(new TemporalCallback(2,0,()->{follower.setMaxPowerScaling(1);}));
        paths.MidSpike.setCallbacks(new TemporalCallback(1,0,()->{follower.setMaxPowerScaling(0.45);}));
        paths.MidSpike.setCallbacks(new TemporalCallback(2,0,()->{follower.setMaxPowerScaling(1);}));
        paths.FrontSpike.setCallbacks(new TemporalCallback(1,0,()->{follower.setMaxPowerScaling(0.45);}));
        paths.FrontSpike.setCallbacks(new TemporalCallback(2,0,()->{follower.setMaxPowerScaling(1);}));
        var startPose = paths.MainChain.getPose(new PathChain.PathT(0, 0));
        var shotPose = paths.MainChain.endPose();
        Robot.drivetrain.setPose(startPose);

        return new SequentialCommandGroup(
                //shot 1
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                //drive to shoot position
                                Robot.drivetrain.followPath(paths.MainChain, false, 1),
                                Robot.helidexer.primeForMotif(),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.preShotRpm(shotPose).perpetually(),
                                Robot.hoodAngle.preShotHood(shotPose).perpetually()
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(500),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.autoShotRpm().perpetually(),
                                Robot.hoodAngle.autoShotHood().perpetually()
                        ),
                        //shoot balls
                        Robot.autoShootMotif()
                ),
                new SequentialCommandGroup(
                        //get back preset balls
                        new ParallelDeadlineGroup(
                                Robot.drivetrain.followPath(paths.BackSpike, false, 1),
                                Robot.commandFloorLoad(),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.preShotRpm(shotPose).perpetually(),
                                Robot.hoodAngle.preShotHood(shotPose).perpetually()
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(500),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.autoShotRpm().perpetually(),
                                Robot.hoodAngle.autoShotHood().perpetually()
                        ),
                        //shoot balls
                        Robot.autoShootMotif()
                ),
                new SequentialCommandGroup(
                        //get mid preset balls
                        new ParallelDeadlineGroup(
                                Robot.drivetrain.followPath(paths.MidSpike, false, 1),
                                Robot.commandFloorLoad(),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.preShotRpm(shotPose).perpetually(),
                                Robot.hoodAngle.preShotHood(shotPose).perpetually()
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(500),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.autoShotRpm().perpetually(),
                                Robot.hoodAngle.autoShotHood().perpetually()
                        ),
                        //shoot balls
                        Robot.autoShootMotif()
                ),
                //front spike
                new SequentialCommandGroup(
                        //get mid preset balls
                        new ParallelDeadlineGroup(
                                Robot.drivetrain.followPath(paths.FrontSpike, false, 1),
                                Robot.commandFloorLoad(),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.preShotRpm(shotPose).perpetually(),
                                Robot.hoodAngle.preShotHood(shotPose).perpetually()
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(500),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.autoShotRpm().perpetually(),
                                Robot.hoodAngle.autoShotHood().perpetually()
                        ),
                        //shoot balls
                        Robot.autoShootMotif()
                ),
                Robot.drivetrain.followPath(paths.End, false, 1)
                //This is a hack that resets the robot back to starting position
                //,Robot.resetRobot(startPose)
        );
    }

    /// START AUTO GENERATED CODE ------------------------------------------------------------------
    public static class Paths {
        public PathChain MainChain;
        public PathChain BackSpike;
        public PathChain MidSpike;
        public PathChain FrontSpike;
        public PathChain End;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(84.865, 7.269),
                                    new Pose(85.500, 22.700)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            BackSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.500, 22.700),
                                    new Pose(97.351, 34.390)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(97.351, 34.390),
                                    new Pose(127.259, 34.707)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.259, 34.707),
                                    new Pose(85.500, 22.700)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            MidSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.500, 22.700),
                                    new Pose(97.193, 57.231)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(97.193, 57.231),
                                    new Pose(122.500, 57.231)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(122.500, 57.231),
                                    new Pose(85.500, 22.700)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            FrontSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.500, 22.700),
                                    new Pose(97.034, 80.914)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(97.034, 80.914),
                                    new Pose(121.830, 80.744)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierCurve(
                                    new Pose(121.830, 80.744),
                                    new Pose(102.965, 71.792),
                                    new Pose(85.500, 22.700)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            End = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.500, 22.700),
                                    new Pose(85.755, 37.362)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }
    /// END AUTO GENERATED CODE --------------------------------------------------------------------
}
