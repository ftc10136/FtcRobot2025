package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;

    public PathChain BlueBackBalls;
    public PathChain BlueBackIntake;
    public PathChain BlueBackToShoot;

    public Paths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(116.000, 131.724),

                                new Pose(105.448, 33.724)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(105.448, 33.724),

                                new Pose(38.276, 33.931)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(38.276, 33.931),

                                new Pose(72.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();

        BlueBackBalls = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.466, 7.672),

                                new Pose(45.424, 35.586)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        BlueBackIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.424, 35.586),

                                new Pose(12.828, 35.586)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        BlueBackToShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.828, 35.586),

                                new Pose(60.000, 23.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

    }
}
