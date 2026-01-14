package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.livoniawarriors.RobotUtil;

public class DrivetrainPP extends SubsystemBase {
    public final Follower follower;
    private final TelemetryPacket packet;

    public DrivetrainPP() {
        super();
        packet = new TelemetryPacket();
        follower = Constants.createFollower(Robot.opMode.hardwareMap);
        follower.setStartingPose(new Pose());
        follower.startTeleopDrive();
    }

    @Override
    public void periodic() {
        follower.update();

        packet.put("Drivetrain/Command", RobotUtil.getCommandName(getCurrentCommand()));
        var pose = follower.getPose();
        packet.put("Drivetrain/Pose x", pose.getX());
        packet.put("Drivetrain/Pose y", pose.getY());
        packet.put("Drivetrain/Pose heading", pose.getHeading());

        Robot.logPacket(packet);
    }

    public Command teleopDrive() {
        return new TeleopDrive();
    }

    class TeleopDrive extends CommandBase {
        public TeleopDrive() {
            addRequirements(Robot.drivetrain);
        }
        @Override
        public void execute() {
            var forward = Robot.controls.getDriveForward();
            var strafe = Robot.controls.getDriveRight();
            var turn = Robot.controls.getDriveTurn();
            follower.setTeleOpDrive(forward, strafe, turn, false);
        }
    }
}
