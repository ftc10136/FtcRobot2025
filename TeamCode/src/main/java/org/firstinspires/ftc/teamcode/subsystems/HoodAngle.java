package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
        shootingTable.put(16., 0.82);
        shootingTable.put(34.5, 0.52);
        shootingTable.put(50., 0.42);
        shootingTable.put(62., 0.35);
        shootingTable.put(82., 0.29);
        shootingTable.put(97., 0.26);
        shootingTable.put(113., 0.23);
        shootingTable.put(125., 0.21);
        shootingTable.put(139., 0.20);
        shootingTable.put(149., 0.18);
        shootingTable.put(160., 0.175);
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

        packet.put("HoodAngle/ServoCommand", hoodAngle.getPosition());
        packet.put("HoodAngle/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("HoodAngle/AtTarget", atTarget());
        Robot.logPacket(packet);
    }

    public boolean atTarget() {
        return atPosTimer <= 0;
    }

    private void setHoodAngle(double position) {
        //get the last servo request
        double curPos = hoodAngle.getPosition();
        //we are assuming that it takes 250ms to get from one side to the other
        double extraTime = Math.abs(position - curPos) * 250;
        atPosTimer = atPosTimer + extraTime;
        hoodAngle.setPosition(position);
    }

    public Command calibrateShot() {
        return new CalibrateShot();
    }

    public Command autoShotHood() {
        return new AutoShotHood();
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
}
