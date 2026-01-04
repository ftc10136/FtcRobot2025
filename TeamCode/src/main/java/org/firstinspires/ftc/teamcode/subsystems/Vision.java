package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;

public class Vision extends SubsystemBase {
    private final Limelight3A limelight;
    private final TelemetryPacket packet;


    public Vision() {
        packet = new TelemetryPacket();
        limelight = Robot.opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        // LimelightPipelines: 1=20/BlueAlliance, 2=24/RedAlliance, 3=20,21,22
        limelight.pipelineSwitch(4);
    }

    @Override
    public void periodic() {
        updateVision();
        var status = limelight.getStatus();
        packet.put("Vision/IsConnected", limelight.isConnected());
        packet.put("Vision/IsRunning", limelight.isRunning());
        packet.put("Vision/FPS", status.getFps());
        Robot.logPacket(packet);
    }

    private void updateVision() {
        var result = limelight.getLatestResult();
        if (result != null) {
            var fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : fiducialResults) {
                var tagId = fiducialResult.getFiducialId();
                packet.put("Vision/TagId", tagId);
                if (tagId == 24) {
                    Pose3D pose = fiducialResult.getCameraPoseTargetSpace();
                    logPose(fiducialResult.getCameraPoseTargetSpace(), "CameraPoseTargetSpace");
                }
            }
        }
    }

    private void logPose(Pose3D pose, String name) {
        //dist is in meters
        var dist = Math.sqrt((pose.getPosition().x * pose.getPosition().x) + (pose.getPosition().y * pose.getPosition().y) + (pose.getPosition().z * pose.getPosition().z));
        //convert to inches
        dist = dist * 39.37;
        packet.put("Vision/" + name, dist);
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
}
