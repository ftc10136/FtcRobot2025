package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;

public class Vision extends SubsystemBase {
    private final Limelight3A limelight;
    private final TelemetryPacket packet;
    private double turretError = 0;
    private double distToTarget = 0;
    private long lastReading = 0;

    private Motifs seenMotif;
    public enum Motifs {
        GPP, //21
        PGP, //22
        PPG  //23
    }
    public Vision() {
        packet = new TelemetryPacket();
        limelight = Robot.opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        // LimelightPipelines: 1=20/BlueAlliance, 2=24/RedAlliance, 3=20,21,22
        limelight.pipelineSwitch(4);
        seenMotif = Motifs.PPG;
    }

    @Override
    public void periodic() {
        updateVision();
        getRobotTranslation();
        var status = limelight.getStatus();
        packet.put("Vision/IsConnected", limelight.isConnected());
        packet.put("Vision/IsRunning", limelight.isRunning());
        packet.put("Vision/CameraConnected", isCameraConnected());
        packet.put("Vision/FPS", status.getFps());
        packet.put("Vision/DistToTarget", distToTarget);
        packet.put("Vision/Motif", seenMotif.name());
        packet.put("Vision/TurretError", getTurretError());
        Robot.logPacket(packet);
        if(isCameraConnected() == false || isCameraConnected() == false) {
            limelight.pipelineSwitch(4);
        }
        Robot.opMode.telemetry.addData("Motif", seenMotif.name());
    }

    private void updateVision() {
        var result = limelight.getLatestResult();
        if (result != null) {
            logPose(result.getBotpose(), "BotPose");
            var fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : fiducialResults) {
                var tagId = fiducialResult.getFiducialId();
                packet.put("Vision/TagId", tagId);
                if ((tagId == 24 && Robot.IsRed) || (tagId == 20 && !Robot.IsRed)) {
                    Pose3D pose = fiducialResult.getCameraPoseTargetSpace();
                    //dist is in meters
                    var dist = Math.sqrt((pose.getPosition().x * pose.getPosition().x) + (pose.getPosition().y * pose.getPosition().y) + (pose.getPosition().z * pose.getPosition().z));
                    //convert to inches
                    distToTarget = dist * 39.37;
                    logPose(fiducialResult.getCameraPoseTargetSpace(), "CameraPoseTargetSpace");
                    turretError = fiducialResult.getTargetXDegrees();
                    lastReading = System.nanoTime();
                }

                if(tagId == 21) {
                    seenMotif = Motifs.GPP;
                } else if(tagId == 22) {
                    seenMotif = Motifs.PGP;
                } else if(tagId == 23) {
                    seenMotif = Motifs.PPG;
                }
            }
        }
    }

    private Pose2D getRobotTranslation() {
        //CONSTANTS
        var OFFSET_ANGLE_DEG = 7.895;
        var CAMERA_TURRET_RADIUS_IN = 3.53553;

        //INPUTS
        var curTurretAngleDeg = 0.;

        //work
        var curTurretAngleRad = Math.toRadians(curTurretAngleDeg + OFFSET_ANGLE_DEG);
        var pose = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        //move it
        pose = pose.plus(new Transform2d(new Translation2d(-0.25,1.75), Rotation2d.fromDegrees(0)));
        return new Pose2D(DistanceUnit.METER, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }

    private void logPose(Pose3D pose, String name) {
        var pedroPose = new Pose(-pose.getPosition().y, pose.getPosition().x, pose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        packet.put("Vision/" + name + " x", pose.getPosition().x);
        packet.put("Vision/" + name + " y", pose.getPosition().y);
        packet.put("Vision/" + name + " heading", pose.getOrientation().getYaw(AngleUnit.RADIANS));

        packet.put("Vision/Pedro " + name + " x", pedroPose.getX()*39.37);
        packet.put("Vision/Pedro " + name + " y", pedroPose.getY()*39.37);
        packet.put("Vision/Pedro " + name + " heading", pedroPose.getHeading()+Math.PI);
    }

    public void initScanning() {
        var status = limelight.getStatus();
        var result = limelight.getLatestResult();
        if (result != null) {
            // Access general information.
            // Access fiducial results.
            var fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult_item : fiducialResults) {
                var fiducialResult = fiducialResult_item;
                //telemetry.addData("Fiducial", "ID: " + fiducialResult.getFiducialId() + ", Family: " + fiducialResult.getFamily() + ", X: " + JavaUtil.formatNumber(fiducialResult.getTargetXDegrees(), 2) + ", Y: " + JavaUtil.formatNumber(fiducialResult.getTargetYDegrees(), 2));
                var AutonMode = fiducialResult.getFiducialId();
                //telemetry.update();
            }
        } else {
            //telemetry.addData("Limelight", "No data available");
        }
    }

    public void teleopScanning() {
        Pose3D botpose;
        double captureLatency;
        double targetingLatency;
        double X_Error;
        double GainFactor;
        double CorrectionNeeded;

        var status = limelight.getStatus();
        var result = limelight.getLatestResult();
        if (result != null) {
            // Access general information.
            var VisionAcquired = 1;
            botpose = result.getBotpose();
            captureLatency = result.getCaptureLatency();
            targetingLatency = result.getTargetingLatency();
            X_Error = result.getTx();
            if (Math.abs(X_Error) < 0.8) {
                GainFactor = 0;
            } else if (Math.abs(X_Error) < 10) {
                GainFactor = 0.8;
            } else {
                GainFactor = 1;
            }
            CorrectionNeeded = X_Error * -0.005 * GainFactor;
            //if (TurretMode == 1) {
            //    TurretSpin.setPosition(TurretSpin.getPosition() - CorrectionNeeded);
            //}
            // Access fiducial results.
            var fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult_item2 : fiducialResults) {
                var fiducialResult = fiducialResult_item2;
                //telemetry.addData("Fiducial", "ID: " + fiducialResult.getFiducialId() + ", Family: " + fiducialResult.getFamily() + ", X: " + JavaUtil.formatNumber(fiducialResult.getTargetXDegrees(), 2) + ", Y: " + JavaUtil.formatNumber(fiducialResult.getTargetYDegrees(), 2));
            }
        } else {
        }
    }

    public double getTurretError() {
        return turretError;
    }

    public double getDistance() {
        return distToTarget;
    }

    public Motifs getSeenMotif() {
        return seenMotif;
    }

    public boolean isCameraConnected() {
        return ((System.nanoTime() - lastReading) < 500_000_000) && limelight.isConnected();
    }
}
