package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.RobotUtil;

import java.util.Optional;

public class Vision extends SubsystemBase {
    public final boolean HUSKYLEN_ENABLED = false;
    public final boolean LIMELIGHT_ENABLED = true;
    private Limelight3A limelight;
    private final TelemetryPacket packet;
    private HuskyLens huskyLens;
    private double distToTarget = 0;
    private long lastReading = 0;
    private HuskyLens.Block[] blocks;
    private Servo limelightServo;
    private Motifs seenMotif;
    private Optional<Pose> visionPose;
    private Optional<Double> targetX;
    private boolean hasMotifTag;

    public enum Motifs {
        Unknown,
        GPP, //21
        PGP, //22
        PPG  //23
    }
    public Vision() {
        packet = new TelemetryPacket();

        if (LIMELIGHT_ENABLED) {
            limelight = Robot.opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            // LimelightPipelines: 1=20/BlueAlliance, 2=24/RedAlliance, 3=20,21,22
            limelight.pipelineSwitch(6);
            limelightServo = Robot.opMode.hardwareMap.get(Servo.class, "CameraServo");
        }
        if (HUSKYLEN_ENABLED) {
            huskyLens = Robot.opMode.hardwareMap.get(HuskyLens.class, "HuskyLens");
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }
        blocks = new HuskyLens.Block[0];

        visionPose = Optional.empty();
        targetX = Optional.empty();
        seenMotif = Motifs.Unknown;
    }

    @Override
    public void periodic() {
        hasMotifTag = false;
        if(LIMELIGHT_ENABLED) {
            if(Robot.isEnabled()) {
                limelightServo.setPosition(Robot.RobotConfig.CAMERA_SERVO_POS);
            }
            //if(!isCameraConnected()) {
                //limelight.pipelineSwitch(6);
            //}
            packet.put("Vision/IsConnected", limelight.isConnected());
            packet.put("Vision/IsRunning", limelight.isRunning());
            updateLimeLight();
        }
        packet.put("Vision/CameraConnected", isCameraConnected());
        packet.put("Vision/DistToTarget", distToTarget);
        packet.put("Vision/Motif", seenMotif.name());
        packet.put("Vision/TargetX", getTargetX());

        if (HUSKYLEN_ENABLED) {
            //blocks = huskyLens.blocks();
            for (int i = 0; i < blocks.length; i++) {
                packet.put("Vision/Block/" + i + "/X", blocks[i].x);
                packet.put("Vision/Block/" + i + "/Y", blocks[i].y);
                packet.put("Vision/Block/" + i + "/id", blocks[i].id);
            }
        }
        Robot.logPacket(packet);
        Robot.opMode.telemetry.addData("Motif", seenMotif.name());
    }

    private void updateLimeLight() {
        var headingDeg = -Robot.drivetrain.getHeading() + 180;
        headingDeg = RobotUtil.inputModulus(headingDeg, -180, 180);
        limelight.updateRobotOrientation(headingDeg);
        visionPose = Optional.empty();
        targetX = Optional.empty();

        var result = limelight.getLatestResult();
        if (result != null) {
            var mt2Pose = result.getBotpose_MT2();
            logPose(result.getBotpose(), "BotPose");
            logPose(mt2Pose, "BotPose_MT2");
            var fiducialResults = result.getFiducialResults();
            boolean hasLocatingTag = false;
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
                    lastReading = System.nanoTime();
                    targetX = Optional.of(fiducialResult.getTargetXDegrees());
                }

                if(tagId == 21) {
                    seenMotif = Motifs.GPP;
                    hasMotifTag = true;
                } else if(tagId == 22) {
                    seenMotif = Motifs.PGP;
                    hasMotifTag = true;
                } else if(tagId == 23) {
                    seenMotif = Motifs.PPG;
                    hasMotifTag = true;
                }

                if(tagId == 20 || tagId == 24) {
                    hasLocatingTag = true;
                }
            }

            if (hasLocatingTag) {
                var pos = mt2Pose.getPosition().toUnit(DistanceUnit.INCH);
                visionPose = Optional.of(new Pose(72 + pos.y, 72 - pos.x, result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS) - (Math.PI / 2), PedroCoordinates.INSTANCE));
            }
        }
    }

    private void logPose(Pose3D pose, String name) {
        var pedroPose = new Pose(-pose.getPosition().y, pose.getPosition().x, pose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        packet.put("Vision/Pedro " + name + " x", pedroPose.getX());
        packet.put("Vision/Pedro " + name + " y", pedroPose.getY());
        packet.put("Vision/Pedro " + name + " heading", pedroPose.getHeading()+Math.PI);
    }

    public Optional<Pose> getVisionPose() {
        return visionPose;
    }

    public Optional<Double> getTargetX() {
        return targetX;
    }

    public double getDistance() {
        return distToTarget;
    }

    public Motifs getSeenMotif() {
        return seenMotif;
    }

    public boolean isCameraConnected() {
        if(LIMELIGHT_ENABLED) {
            return ((System.nanoTime() - lastReading) < 500_000_000) && limelight.isConnected();
        }
        return false;
    }

    public HuskyLens.Block[] getBlocks() {
        return blocks;
    }

    public boolean seesBalls() {
        return blocks.length > 0;
    }
    public boolean seesMotif() {return hasMotifTag;}
}
