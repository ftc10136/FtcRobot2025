package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.TemporalCallback;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;

public class RedFrontBase {
    Paths paths;
    Follower follower;

    public RedFrontBase() {
        follower = Robot.drivetrain.getFollower();
        paths = new Paths(follower);
    }


    public Pose GetStartPose() {
        return paths.MainChain.getPose(new PathChain.PathT(0, 0));
    }

    public Command LeaveStartAndShoot() {
        var shotPose = paths.MainChain.endPose();
        return new SequentialCommandGroup(
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
        );
    }

    public Command FrontSpike() {
        var shotPose = paths.MainChain.endPose();
        paths.FrontSpike.setCallbacks(new TemporalCallback(0, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.FrontSpike.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.4)
        ));
        paths.FrontSpike.setCallbacks(new TemporalCallback(2, 0, () ->
                follower.setMaxPowerScaling(1)
        ));
        return new SequentialCommandGroup(
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
        );
    }

    public Command MidSpike() {
        //TODO remove Gate
        var shotPose = paths.MainChain.endPose();
        paths.MidSpike.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpike.setCallbacks(new TemporalCallback(2, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpike.setCallbacks(new TemporalCallback(3, 0, () ->
                follower.setMaxPowerScaling(1)
        ));
        return new SequentialCommandGroup(
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
        );
    }

    public Command MidSpikeAndGate() {
        //TODO implement
        return new InstantCommand();/*
        var shotPose = paths.MainChain.endPose();
        paths.MidSpike.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpike.setCallbacks(new TemporalCallback(2, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpike.setCallbacks(new TemporalCallback(3, 0, () ->
                follower.setMaxPowerScaling(1)
        ));
        return new SequentialCommandGroup(
                //get mid preset balls
                new ParallelDeadlineGroup(
                        Robot.drivetrain.followPath(paths.MidSpikeAndGate, false, 1),
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
        );*/
    }
    public Command MidRamp() {
        //TODO implement
        return new InstantCommand();/*
        var shotPose = paths.MainChain.endPose();
        return new SequentialCommandGroup(
                //get mid ramp
                Robot.drivetrain.followPath(paths.SpotToRamp, true, 1),
                Robot.commandFloorLoad().withTimeout(5000),
                //return to spot
                new ParallelDeadlineGroup(
                        Robot.drivetrain.followPath(paths.RampToSpot, true, 1),
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
        );*/
    }

    public Command FrontSpikeAndGate() {
        //TODO imprlemtn
        return new InstantCommand();/*
        var shotPose = paths.MainChain.endPose();
        paths.FrontSpikeAndRamp.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.FrontSpikeAndRamp.setCallbacks(new TemporalCallback(2, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.FrontSpikeAndRamp.setCallbacks(new TemporalCallback(3, 0, () -> {
            follower.setMaxPowerScaling(1);
        }
        ));
        return new SequentialCommandGroup(
                //get mid preset balls
                new ParallelDeadlineGroup(
                        Robot.drivetrain.followPath(paths.FrontSpikeAndRamp, false, 1),
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
        );*/
    }

    public Command BackSpike() {
        var shotPose = paths.MainChain.endPose();
        paths.RearSpike.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.4)
        ));
        paths.RearSpike.setCallbacks(new TemporalCallback(2, 0, () ->
                follower.setMaxPowerScaling(1)
        ));
        return new SequentialCommandGroup(
                //get mid preset balls
                new ParallelDeadlineGroup(
                        Robot.drivetrain.followPath(paths.RearSpike, false, 1),
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
        );
    }

    public Command LeaveShotSpot() {
        //TODO FIX ME
        return new InstantCommand();
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
