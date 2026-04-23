package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.RobotUtil;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class HoodAngle extends SubsystemBase {
    private final Servo hoodAngle;
    private final TelemetryPacket packet;
    private final InterpolatingDoubleTreeMap shootingTable;
    private long lastLoopTime;
    private double atPosTimer;
    private double lastCommand = 0;

    public HoodAngle() {
        packet = new TelemetryPacket();
        hoodAngle = Robot.opMode.hardwareMap.get(Servo.class, "HoodAngle");
        hoodAngle.scaleRange(0.2, 0.8);
        lastLoopTime = System.nanoTime();
        setHoodAngle(0.35);
        atPosTimer = 0;

        //hood angle table, input distance in inches, output hood angle command
        //values are from testing calibrateShot command at different distances
        //also set HoodAngle.shootingTable for hood angles
        shootingTable = new InterpolatingDoubleTreeMap();
        shootingTable.put(35.35, .88);
        shootingTable.put(47.93, .58);
        shootingTable.put(60.1, .47);
        shootingTable.put(67.59, .4); //48"
        shootingTable.put(79.8, .37); //60"
        shootingTable.put(92.09, .32); //72"
        shootingTable.put(104.78, .35); //84
        shootingTable.put(116.58, .3); //96
        shootingTable.put(132.33, .26); //108
        shootingTable.put(143.21, .22); //120
        shootingTable.put(151.91, .22); //132
        shootingTable.put(164.51, .22); //144
        shootingTable.put(175.52, .22); //156
    }

    @Override
    public void periodic() {
        //handle timer to know if we are ready to shoot
        long curTime = System.nanoTime();
        atPosTimer = atPosTimer - ((curTime - lastLoopTime) / 1000000.);
        lastLoopTime = curTime;
        if(atPosTimer < 0) {
            atPosTimer = 0;
        }

        packet.put("HoodAngle/ServoCommand", lastCommand);
        packet.put("HoodAngle/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("HoodAngle/AtTarget", atTarget());
        packet.put("HoodAngle/Timer", atPosTimer);
        Robot.logPacket(packet);
    }

    public boolean atTarget() {
        //can't be 0 since this is float math and might not be real zero
        return atPosTimer <= 0.01;
    }

    private void setHoodAngle(double position) {
        //get the last servo request
        double curPos = lastCommand;
        //we are assuming that it takes 250ms to get from one side to the other
        double extraTime = Math.abs(position - curPos) * 250;
        atPosTimer = atPosTimer + extraTime;
        hoodAngle.setPosition(position);
        lastCommand = position;
    }

    public Command calibrateShot() {
        return new CalibrateShot();
    }

    public Command autoShotHood() {
        return new AutoShotHood();
    }
    public Command preShotHood(Pose shootingPose) {
        return new PreShotHood(shootingPose);
    }

    private class CalibrateShot extends CommandBase {
        public CalibrateShot() {
            addRequirements(Robot.hoodAngle);
        }
        @Override
        public void execute() {
            setHoodAngle(Robot.RobotConfig.CALIBRATE_SHOT_HOOD);
        }
    }

    private class AutoShotHood extends CommandBase {
        public AutoShotHood() {
            addRequirements(Robot.hoodAngle);
        }
        @Override
        public void execute() {
            double dist = Robot.drivetrain.getGoalDistance();
            var angle = shootingTable.get(dist);
            setHoodAngle(angle);
        }
    }

    private class PreShotHood extends CommandBase {
        Pose pose;
        double angle;
        public PreShotHood(Pose shootingPose) {
            pose = shootingPose;
            addRequirements(Robot.hoodAngle);
        }
        @Override
        public void initialize() {
            double dist = DrivetrainPP.getGoalTarget(pose).GoalDistance;
            angle = shootingTable.get(dist);
            setHoodAngle(angle);
        }
        @Override
        public void execute() {
            setHoodAngle(angle);
        }
    }
}
