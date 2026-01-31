package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.livoniawarriors.RobotUtil;

public class DrivetrainPP extends SubsystemBase {
    public final Follower follower;
    private final TelemetryPacket packet;
    private double headingZeroRad;

    public DrivetrainPP() {
        super();
        packet = new TelemetryPacket();
        follower = Constants.createFollower(Robot.opMode.hardwareMap);
        follower.setStartingPose(new Pose());
        follower.startTeleopDrive(true);
        headingZeroRad = 0;
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

    public Command resetFieldOriented() {
        return new CommandBase() {
            @Override
            public void execute() {
                headingZeroRad = follower.getHeading();
                if(!Robot.IsRed) {
                    //if we are blue, the heading changes 180*
                    headingZeroRad = Math.PI + headingZeroRad;
                }
            }
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    class TeleopDrive extends CommandBase {
        public TeleopDrive() {
            addRequirements(Robot.drivetrain);
        }
        @Override
        public void execute() {
            //this logic mostly works without AprilTags, might break when the robot knows where it is...
            double Drive2 = Range.clip(Math.sqrt(Math.pow(Robot.controls.getDriveForward(), 2) + Math.pow(Robot.controls.getDriveRight(), 2)), 0, 1);
            double GamePadDegree = Math.atan2(-Math.pow(Robot.controls.getDriveForward(), 3), Math.pow(Robot.controls.getDriveRight(), 3)) / Math.PI * 180;
            double Movement = RobotUtil.inputModulus(GamePadDegree + Math.toDegrees(follower.getHeading()-headingZeroRad),-179.9999, 180);
            double Strafe = Math.cos(Movement / 180 * Math.PI) * Drive2;
            double Forward = Math.sin(Movement / 180 * Math.PI) * Drive2;
            var turn = Robot.controls.getDriveTurn();
            follower.setTeleOpDrive(Forward, Strafe, turn, true);
        }
    }
}
