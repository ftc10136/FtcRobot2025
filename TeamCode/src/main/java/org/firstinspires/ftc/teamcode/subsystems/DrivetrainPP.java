package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.livoniawarriors.RobotUtil;

import java.util.HashMap;

public class DrivetrainPP extends SubsystemBase {
    public final Follower follower;
    private final TelemetryPacket packet;
    private final GoBildaPinpointDriver odo;

    //for logging only
    private HashMap<String, DcMotorEx> motors;

    public DrivetrainPP() {
        super();
        packet = new TelemetryPacket();
        follower = Constants.createFollower(Robot.opMode.hardwareMap);
        odo = Robot.opMode.hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        RobotState.headingZeroRad = 0;

        motors = new HashMap<>();
        motors.put("leftFront", Robot.opMode.hardwareMap.get(DcMotorEx.class, "leftFront"));
        motors.put("leftRear", Robot.opMode.hardwareMap.get(DcMotorEx.class, "leftRear"));
        motors.put("rightFront", Robot.opMode.hardwareMap.get(DcMotorEx.class, "rightFront"));
        motors.put("rightRear", Robot.opMode.hardwareMap.get(DcMotorEx.class, "rightRear"));
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return RobotState.robotPose;
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
        RobotState.robotPose = follower.getPose();

        packet.put("Drivetrain/Command", RobotUtil.getCommandName(getCurrentCommand()));
        var pose = RobotState.robotPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        packet.put("Drivetrain/Pose x", -pose.getX());
        packet.put("Drivetrain/Pose y", -pose.getY());
        packet.put("Drivetrain/Pose heading", pose.getHeading()+Math.PI);
        var pedroPose = RobotState.robotPose;
        packet.put("Drivetrain/PedroX", pedroPose.getX());
        packet.put("Drivetrain/PedroY", pedroPose.getY());
        packet.put("Drivetrain/PedroHeading", pedroPose.getHeading());
        var reading = getGoalTarget(getPose());
        packet.put("Drivetrain/GoalDist", reading.GoalDistance);
        packet.put("Drivetrain/TurretToPlateDist", reading.TagDistance);
        packet.put("Drivetrain/GoalAngle", reading.GoalAngle);

        for(var motorName : motors.keySet()) {
            var motor = motors.get(motorName);
            packet.put("Drivetrain/" + motorName + "/Power", motor.getPower());
            packet.put("Drivetrain/" + motorName + "/Velocity", motor.getVelocity());
            packet.put("Currents/" + motorName, motor.getCurrent(CurrentUnit.AMPS));
        }

        if (Robot.controls.resetPose()) {
            //resetPose();
        }

        Robot.logPacket(packet);
    }

    public void setPose(Pose pose) {
        RobotState.robotPose = pose;
        follower.setPose(pose);
    }

    public boolean resetPose() {
        var visionPose = Robot.vision.getVisionPose();
        if (visionPose.isPresent()) {
            Robot.opMode.telemetry.addLine("PedroPose: " + getPose());
            Robot.opMode.telemetry.addLine("VisionPose: " + Robot.vision.getVisionPose());
            setPose(visionPose.get());
            return true;
        }
        return false;
    }

    public Command resetPoseCommand() {
        return new CommandBase() {
            boolean finished = false;
            int loops = 0;

            @Override
            public void initialize() {
                loops = 10;
            }

            @Override
            public void execute() {
                if(resetPose()) {
                    loops--;
                }
            }

            @Override
            public boolean isFinished() {
                return loops <= 0;
            }
        };
    }

    public Command teleopDrive() {
        return new TeleopDrive();
    }

    public Command holdAtSpot() {
        return new HoldAtSpot();
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
                RobotState.headingZeroRad = follower.getHeading();
                if(!Robot.IsRed) {
                    //if we are blue, the heading changes 180*
                    RobotState.headingZeroRad += Math.PI;
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
        public void initialize() {
            follower.startTeleOpDrive(true);
        }
        @Override
        public void execute() {
            //this logic mostly works without AprilTags, might break when the robot knows where it is...
            double Drive2 = Range.clip(Math.sqrt(Math.pow(Robot.controls.getDriveForward(), 2) + Math.pow(Robot.controls.getDriveRight(), 2)), 0, 1);
            if(Robot.controls.slowModeActive()) {
                Drive2 *= 0.3;
            }
            double GamePadDegree = Math.atan2(-Math.pow(Robot.controls.getDriveForward(), 3), Math.pow(Robot.controls.getDriveRight(), 3)) / Math.PI * 180;
            double Movement = RobotUtil.inputModulus(GamePadDegree + Math.toDegrees(follower.getHeading()-RobotState.headingZeroRad),0, 360);
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
        return getGoalTarget(RobotState.robotPose).GoalDistance;
    }

    public static class DistanceResults {
        double TurretPoseX;
        double TurretPoseY;
        /// What angle to shoot at the goal, 0* north, cw increments in degrees
        double GoalAngle;
        /// How far is the target away in inches
        double GoalDistance;
        double TagDistance;
    }

    public static DistanceResults getGoalTarget(Pose robotPose) {
        double CAMERA_X = 4;
        double CAMERA_Y = 0;
        DistanceResults results = new DistanceResults();

        //calculate where the turret is based on the rotation of the robot
        double hyp = Math.sqrt(CAMERA_X * CAMERA_X + CAMERA_Y * CAMERA_Y);
        double cameraRad = Math.asin(CAMERA_X / hyp);
        //Pedro coordinates are 0* to the east, ccw increments, we use 0* north, cw increments
        double headRad = Math.PI/2 - robotPose.getHeading() + cameraRad;

        results.TurretPoseX = robotPose.getX() + hyp * Math.sin(headRad);
        results.TurretPoseY = robotPose.getY() + hyp * Math.cos(headRad);

        //currently, we are always shooting at the back corner of the goals, at logo height to bounce off
        double goalX, goalY, tagX, tagY;
        if (Robot.IsRed) {
            goalX = 140;
            tagX = 129;
            tagY = 131;
        } else {
            goalX = 0;
            tagX = 15;
            tagY = 131;
        }
        goalY = 144;

        //calculate the goal offsets
        double deltaX = goalX - results.TurretPoseX;
        double deltaY = goalY - results.TurretPoseY;
        //double deltaX = goalX - robotPose.getX();
        //double deltaY = goalY - robotPose.getY();
        results.GoalDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        results.GoalAngle = Math.toDegrees(Math.atan(deltaX / deltaY));

        deltaX = tagX - results.TurretPoseX;
        deltaY = tagY - results.TurretPoseY;
        results.TagDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        return results;
    }

    public double getGoalAngle() {
        return getGoalTarget(RobotState.robotPose).GoalAngle;
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
        boolean hasBalls;
        public DriveToBall() {
            hasBalls = false;
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
                hasBalls = false;
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
                forward = (240-block.y) * 0.00416;
                //for turn, we want to aim to put the center of the block at the center of the camera
                turn = (160-block.x) * 0.006;
                hasBalls = true;
            }
            follower.setTeleOpDrive(forward, 0, turn, true);
        }
        @Override
        public boolean isFinished() {
            return hasBalls;
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

    class HoldAtSpot extends CommandBase {
        private Pose holdPose;
        public HoldAtSpot() {

        }
        @Override
        public void initialize() {
            holdPose = getPose();
        }
        @Override
        public void execute() {
            follower.holdPoint(holdPose);
        }
        @Override
        public boolean isFinished() {
            return true;
        }
        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
        }
    }
}
