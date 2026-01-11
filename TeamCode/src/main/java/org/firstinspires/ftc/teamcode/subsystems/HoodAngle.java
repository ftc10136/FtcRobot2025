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

    public HoodAngle() {
        packet = new TelemetryPacket();
        hoodAngle = Robot.opMode.hardwareMap.get(Servo.class, "HoodAngle");
        hoodAngle.scaleRange(0.2, 0.8);
        hoodAngle.setPosition(0.35);

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
        //TODO, need to get table to 144", to get the back corner shots
    }

    @Override
    public void periodic() {
        packet.put("HoodAngle/ServoCommand", hoodAngle.getPosition());
        packet.put("HoodAngle/Command", RobotUtil.getCommandName(getCurrentCommand()));
        Robot.logPacket(packet);
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
            hoodAngle.setPosition(Robot.RobotConfig.CALIBRATE_SHOT_HOOD);
        }
    }

    private class AutoShotHood extends CommandBase {
        public AutoShotHood() {
            addRequirements(Robot.hoodAngle);
        }
        @Override
        public void execute() {
            double dist = Robot.vision.getDistance();
            var angle = shootingTable.get(dist);
            hoodAngle.setPosition(angle);
        }
    }
}
