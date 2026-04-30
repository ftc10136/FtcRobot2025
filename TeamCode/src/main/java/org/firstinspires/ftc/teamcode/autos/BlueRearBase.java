package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.TemporalCallback;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;

public class BlueRearBase {
    Paths paths;
    Follower follower;

    public BlueRearBase() {
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
        paths.FrontSpike.setCallbacks(new TemporalCallback(1,0,()->{follower.setMaxPowerScaling(0.45);}));
        paths.FrontSpike.setCallbacks(new TemporalCallback(2,0,()->{follower.setMaxPowerScaling(1);}));
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
        paths.MidSpike.setCallbacks(new TemporalCallback(1,0,()->{follower.setMaxPowerScaling(0.45);}));
        paths.MidSpike.setCallbacks(new TemporalCallback(2,0,()->{follower.setMaxPowerScaling(1);}));
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

    public Command BackSpike() {
        var shotPose = paths.MainChain.endPose();
        paths.BackSpike.setCallbacks(new TemporalCallback(1,0,()->{follower.setMaxPowerScaling(0.45);}));
        paths.BackSpike.setCallbacks(new TemporalCallback(2,0,()->{follower.setMaxPowerScaling(1);}));
        return new SequentialCommandGroup(
                //get mid preset balls
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
        );
    }

    public Command BackBall() {
        Pose lookPose = new Pose(45, 11, Math.toRadians(180));
        Pose shootPose = paths.MainChain.endPose();;

        return new SequentialCommandGroup(
                Robot.drivetrain.DriveToPose(lookPose, false, 1),
                new ParallelDeadlineGroup(
                        Robot.commandFloorLoad(),
                        new RepeatCommand(new SequentialCommandGroup(
                                Robot.drivetrain.driveToBall(),
                                Robot.drivetrain.DriveToPose(lookPose, false, 1).interruptOn(()->Robot.vision.seesBalls())
                        )),
                        Robot.turret.centerTurretViaPosition().perpetually(),
                        Robot.shooter.preShotRpm(shootPose).perpetually(),
                        Robot.hoodAngle.preShotHood(shootPose).perpetually()
                ).withTimeout(10000),
                new ParallelDeadlineGroup(
                        Robot.drivetrain.DriveToPose(shootPose, true,1),
                        Robot.turret.centerTurretViaPosition().perpetually(),
                        Robot.shooter.preShotRpm(shootPose).perpetually(),
                        Robot.hoodAngle.preShotHood(shootPose).perpetually()
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
        return Robot.drivetrain.followPath(paths.End, false, 1);
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
                                    new Pose(56.635, 9.269),
                                    new Pose(56.000, 24.700)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            BackSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 24.700),
                                    new Pose(41.610, 35.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(41.610, 35.500),
                                    new Pose(15.034, 34.865)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(15.034, 34.865),
                                    new Pose(56.000, 24.700)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            MidSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 24.700),
                                    new Pose(40.500, 58.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(40.500, 58.500),
                                    new Pose(19.000, 58.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(19.000, 58.500),
                                    new Pose(56.000, 24.700)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            FrontSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 24.700),
                                    new Pose(40.500, 82.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(40.500, 82.500),
                                    new Pose(17.608, 82.330)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierCurve(
                                    new Pose(17.608, 82.330),
                                    new Pose(38.535, 71.792),
                                    new Pose(56.000, 24.700)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            End = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 24.700),
                                    new Pose(55.745, 37.362)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
    /// END AUTO GENERATED CODE --------------------------------------------------------------------
}
