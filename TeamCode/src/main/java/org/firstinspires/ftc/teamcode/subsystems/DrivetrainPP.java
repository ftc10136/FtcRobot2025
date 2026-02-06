package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.livoniawarriors.RobotUtil;

public class DrivetrainPP extends SubsystemBase {
    public final Follower follower;
    private final TelemetryPacket packet;
    private double headingZeroRad;
    private final GoBildaPinpointDriver odo;

    public DrivetrainPP() {
        super();
        packet = new TelemetryPacket();
        follower = Constants.createFollower(Robot.opMode.hardwareMap);
        odo = Robot.opMode.hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        follower.setStartingPose(new Pose(0,0,0,FTCCoordinates.INSTANCE));
        follower.startTeleopDrive(true);
        headingZeroRad = 0;
    }

    public Follower getFollower() {
        return follower;
    }

    @Override
    public void periodic() {
        follower.update();

        packet.put("Drivetrain/Command", RobotUtil.getCommandName(getCurrentCommand()));
        var pose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        packet.put("Drivetrain/Pose x", -pose.getX());
        packet.put("Drivetrain/Pose y", -pose.getY());
        packet.put("Drivetrain/Pose heading", pose.getHeading()+Math.PI);
        packet.put("Drivetrain/EncoderX", odo.getEncoderX());
        packet.put("Drivetrain/EncoderY", odo.getEncoderY());

        Robot.logPacket(packet);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Command teleopDrive() {
        return new TeleopDrive();
    }
    public Command followPath(PathChain path) {
        return new FollowPathCommand(follower, path);
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
            double Movement = RobotUtil.inputModulus(GamePadDegree + Math.toDegrees(follower.getHeading()-headingZeroRad),0, 360);
            double Strafe = Math.cos(Movement / 180 * Math.PI) * Drive2;
            double Forward = Math.sin(Movement / 180 * Math.PI) * Drive2;
            var turn = Robot.controls.getDriveTurn();
            follower.setTeleOpDrive(Forward, Strafe, turn, true);
        }
    }

    class DrivePath extends CommandBase {
        private final PathChain path;
        public DrivePath(PathChain path) {
            this.path = path;
            addRequirements(Robot.drivetrain);
        }
        @Override
        public void initialize() {
            follower.followPath(path,true);
        }
        @Override
        public void execute() {
            follower.update();
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
        }
    }
}
