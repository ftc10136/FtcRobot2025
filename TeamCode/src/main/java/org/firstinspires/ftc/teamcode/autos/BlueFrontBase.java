package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.TemporalCallback;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;

public class BlueFrontBase {
    Paths paths;
    Follower follower;

    public BlueFrontBase() {
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
        var shotPose = paths.MainChain.endPose();
        paths.MidSpike.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpike.setCallbacks(new TemporalCallback(2, 0, () ->
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
        var shotPose = paths.MainChain.endPose();
        paths.MidSpikeAndGate.setCallbacks(new TemporalCallback(1, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpikeAndGate.setCallbacks(new TemporalCallback(2, 0, () ->
                follower.setMaxPowerScaling(0.6)
        ));
        paths.MidSpikeAndGate.setCallbacks(new TemporalCallback(3, 0, () -> {
            follower.setMaxPowerScaling(1);
        }));return new SequentialCommandGroup(
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
        );
    }
    public Command MidRamp() {
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
        );
    }

    public Command FrontSpikeAndGate() {
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
        );
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
        return Robot.drivetrain.followPath(paths.LeaveShotSpot, false, 1);
    }


    /// START AUTO GENERATED CODE ------------------------------------------------------------------
    public static class Paths {
        public PathChain MainChain;
        public PathChain FrontSpike;
        public PathChain RearSpike;
        public PathChain MidSpike;
        public PathChain MidSpikeAndGate;
        public PathChain FrontSpikeAndRamp;
        public PathChain LeaveShotSpot;
        public PathChain SpotToRamp;
        public PathChain RampToSpot;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(17.778, 119.914),
                                    new Pose(59.700, 77.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143.7), Math.toRadians(180))
                    .build();

            FrontSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),
                                    new Pose(39.221, 82.503)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(39.221, 82.503),
                                    new Pose(18.469, 82.255)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.469, 82.255),
                                    new Pose(59.700, 77.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            RearSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),
                                    new Pose(42.164, 34.707)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(42.164, 34.707),
                                    new Pose(17.517, 34.865)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(17.517, 34.865),
                                    new Pose(54.441, 110.187)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MidSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),
                                    new Pose(39.817, 58.562)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(39.817, 58.562),
                                    new Pose(10.809, 58.086)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.809, 58.086),
                                    new Pose(31.889, 57.735),
                                    new Pose(59.700, 77.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            MidSpikeAndGate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),
                                    new Pose(39.817, 58.562)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(39.817, 58.562),
                                    new Pose(10.809, 58.086)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.809, 58.086),
                                    new Pose(33.783, 61.938),
                                    new Pose(17.700, 67.845)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(17.700, 67.845),
                                    new Pose(59.700, 77.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            FrontSpikeAndRamp = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),
                                    new Pose(39.200, 82.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(39.200, 82.500),
                                    new Pose(18.400, 82.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.400, 82.500),
                                    new Pose(26.100, 73.000),
                                    new Pose(17.700, 71.841)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(17.700, 71.841),
                                    new Pose(59.700, 77.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            LeaveShotSpot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(54.441, 110.187),
                                    new Pose(59.700, 77.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),
                                    new Pose(21.246, 64.335)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                    .build();

            SpotToRamp = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(21.246, 64.335),
                                    new Pose(59.700, 77.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(59.700, 77.400),
                                    new Pose(8.400, 60.300)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145.5))
                    .build();

            RampToSpot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.400, 60.300),
                                    new Pose(59.700, 77.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145.5), Math.toRadians(180))
                    .build();
        }
    }
    /// END AUTO GENERATED CODE --------------------------------------------------------------------
}
