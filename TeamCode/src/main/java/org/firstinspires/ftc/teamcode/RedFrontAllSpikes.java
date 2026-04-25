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
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

@Autonomous
public class RedFrontAllSpikes extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.runAutonomous(this, this::getAutoSequence, true);
    }

    private Command getAutoSequence() {
        var follower = Robot.drivetrain.getFollower();
        Paths paths = new Paths(follower);
        paths.MidSpike.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpike.setCallbacks(new TemporalCallback(2, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpike.setCallbacks(new TemporalCallback(3, 0, () ->
            follower.setMaxPowerScaling(1)
        ));
        paths.FrontSpike.setCallbacks(new TemporalCallback(0, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.FrontSpike.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.4)
        ));
        paths.FrontSpike.setCallbacks(new TemporalCallback(2, 0, () ->
            follower.setMaxPowerScaling(1)
        ));
        paths.RearSpike.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.4)
        ));
        paths.RearSpike.setCallbacks(new TemporalCallback(2, 0, () ->
                follower.setMaxPowerScaling(1)
        ));
        var startPose = paths.MainChain.getPose(new PathChain.PathT(0, 0));
        var shotPose = paths.MainChain.endPose();
        var shotEndPose = paths.RearSpike.endPose();
        Robot.drivetrain.setPose(startPose);

        return new SequentialCommandGroup(
                //shot 1
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                //drive to shoot position
                                Robot.drivetrain.followPath(paths.MainChain, false, 1),
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
                new SequentialCommandGroup(
                        //get front preset balls
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
                //front spike
                new SequentialCommandGroup(
                        //get rear preset balls
                        new ParallelDeadlineGroup(
                                Robot.drivetrain.followPath(paths.RearSpike, false, 1),
                                Robot.commandFloorLoad(),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.preShotRpm(shotEndPose).perpetually(),
                                Robot.hoodAngle.preShotHood(shotEndPose).perpetually()
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(500),
                                Robot.turret.centerTurretViaPosition().perpetually(),
                                Robot.shooter.autoShotRpm().perpetually(),
                                Robot.hoodAngle.autoShotHood().perpetually()
                        ),
                        //shoot balls
                        Robot.autoShootMotif()
                )
                //This is a hack that resets the robot back to starting position
                //,Robot.resetRobot(startPose)
        );
    }

    /// START AUTO GENERATED CODE ------------------------------------------------------------------
    public static class Paths {
        public PathChain MainChain;
        public PathChain FrontSpike;
        public PathChain RearSpike;
        public PathChain MidSpike;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.722, 119.914),
                                    new Pose(81.800, 77.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36.3), Math.toRadians(0))
                    .build();

            FrontSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(81.800, 77.400),
                                    new Pose(102.279, 82.503)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(102.279, 82.503),
                                    new Pose(123.031, 82.255)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.031, 82.255),
                                    new Pose(81.800, 77.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            RearSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(81.800, 77.400),
                                    new Pose(102.336, 34.707)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(102.336, 34.707),
                                    new Pose(123.983, 34.865)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.983, 34.865),
                                    new Pose(87.059, 110.187)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MidSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(81.800, 77.400),
                                    new Pose(101.683, 58.562)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(101.683, 58.562),
                                    new Pose(130.691, 58.086)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.691, 58.086),
                                    new Pose(107.717, 61.938),
                                    new Pose(120.669, 67.845)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(120.669, 67.845),
                                    new Pose(81.800, 77.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
    /// END AUTO GENERATED CODE --------------------------------------------------------------------
}
