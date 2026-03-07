package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
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
        headingZeroRad = 0;
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void startTeleopDrive(boolean isTeleop) {
        if(isTeleop) {
            follower.startTeleopDrive(true);
        } else {
            follower.breakFollowing();
        }
    }

    @Override
    public void periodic() {
        follower.update();

        packet.put("Drivetrain/Command", RobotUtil.getCommandName(getCurrentCommand()));
        var pose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        packet.put("Drivetrain/Pose x", -pose.getX());
        packet.put("Drivetrain/Pose y", -pose.getY());
        packet.put("Drivetrain/Pose heading", pose.getHeading()+Math.PI);
        var pedroPose = follower.getPose();
        packet.put("Drivetrain/PedroX", pedroPose.getX());
        packet.put("Drivetrain/PedroY", pedroPose.getY());
        packet.put("Drivetrain/PedroHeading", pedroPose.getHeading());
        packet.put("Drivetrain/EncoderX", odo.getEncoderX());
        packet.put("Drivetrain/EncoderY", odo.getEncoderY());
        packet.put("Drivetrain/GoalDist", getGoalDistance());
        packet.put("Drivetrain/GoalAngle", getGoalAngle());

        Robot.logPacket(packet);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Command teleopDrive() {
        return new TeleopDrive();
    }
    public Command followPath(PathChain path) {
        return followPath(path, true);
    }
    public Command followPath(PathChain path, boolean stopAtEnd) {
        return followPath(path, stopAtEnd, 0.8);
    }
    public Command followPath(PathChain path, boolean stopAtEnd, double maxSpeed) {
        return new DrivePath(path, stopAtEnd, maxSpeed);
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
        boolean holdEnd;
        double maxPower;

        public DrivePath(PathChain path, boolean holdEnd, double maxPower) {
            this.path = path;
            this.holdEnd = holdEnd;
            this.maxPower = maxPower;
            addRequirements(Robot.drivetrain);
        }
        @Override
        public void initialize() {
            follower.followPath(path,maxPower,holdEnd);
        }
        @Override
        public void execute() {
            //follower.update();
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

    public double getGoalDistance() {
        if(Robot.vision.isCameraConnected()) {
            return Robot.vision.getDistance();
        } else {
            Pose goal;
            if (Robot.IsRed) {
                goal = new Pose(128, 132, 0, PedroCoordinates.INSTANCE);
            } else {
                goal = new Pose(16, 132, 0, PedroCoordinates.INSTANCE);
            }
            var deltaX = goal.getX() - follower.getPose().getX();
            var deltaY = goal.getY() - follower.getPose().getY();
            return Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
        }
    }

    public double getGoalAngle() {
        Pose goal;
        if(Robot.IsRed) {
            goal = new Pose(132,140,0, PedroCoordinates.INSTANCE);
        } else {
            goal = new Pose(12,140,0, PedroCoordinates.INSTANCE);
        }
        var deltaX = goal.getX() - follower.getPose().getX();
        var angleRad = Math.acos(deltaX / getGoalDistance());
        return 90-Math.toDegrees(angleRad);
    }

    //zero = look at goal side, clockwise positive
    public double getHeading() {
        double newAngle = 90 - Math.toDegrees(follower.getHeading());
        return RobotUtil.inputModulus(newAngle, -180, 180);
    }

    public Command driveToBall() {
        return new DriveToBall();
    }
    class DriveToBall extends CommandBase {
        public DriveToBall() {
            addRequirements(Robot.drivetrain);
        }

        @Override
        public void execute() {
            double forward, turn;
            var blocks = Robot.vision.getBlocks();
            if(blocks.length == 0) {
                //if we don't see a ball, turn till we see one
                forward = 0;
                if(Robot.IsRed) {
                    turn = 0.2;
                } else {
                    turn = -0.2;
                }
            } else {
                //find the most center block
                var block = blocks[0];
                for(int i=0; i< blocks.length; i++) {
                    if(Math.abs(block.x - 160) > Math.abs(blocks[i].x - 160)) {
                        block = blocks[i];
                    }
                }
                //the coordinates are 0-320 x, 0-240 y, where top left is 0,0
                //for forward, the closer we are to a ball, drive slower
                forward = (240-block.y) * 0.0025;
                //for turn, we want to aim to put the center of the block at the center of the camera
                turn = (160-block.x) * 0.005;
            }
            follower.setTeleOpDrive(forward, 0, turn, true);
        }
    }

    public Command DriveToPose(Pose pose) {
        return DriveToPose(pose, true);
    }
    public Command DriveToPose(Pose pose, boolean holdEnd) {
        return DriveToPose(pose, holdEnd, 0.8);
    }
    public Command DriveToPose(Pose pose, boolean holdEnd, double maxPower) {
        return new DriveToPose(pose, holdEnd, maxPower);
    }

    class DriveToPose extends CommandBase {
        Pose pose;
        boolean holdEnd;
        double maxPower;

        public DriveToPose(Pose pose, boolean holdEnd, double maxPower) {
            this.pose = pose;
            this.holdEnd = holdEnd;
            this.maxPower = maxPower;
            addRequirements(Robot.drivetrain);
        }
        @Override
        public void initialize() {
            var curPose = getPose();
            var path = follower.pathBuilder()
                    .addPath(new BezierLine(curPose, pose))
                    .setLinearHeadingInterpolation(curPose.getHeading(), pose.getHeading())
                    .build();
            follower.followPath(path,maxPower,holdEnd);
        }
        @Override
        public void execute() {}
        @Override
        public boolean isFinished() {
            return follower.atPose(pose, 1, 1, 0.3);
        }
        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
            follower.startTeleOpDrive(holdEnd);
        }
    }
}
