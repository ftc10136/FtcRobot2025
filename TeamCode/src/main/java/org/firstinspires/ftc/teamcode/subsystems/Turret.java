package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotState;
import org.livoniawarriors.RobotUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class Turret extends SubsystemBase {
    private final TelemetryPacket packet;
    private final AnalogInput turretEncoder;
    private final Servo turretSpin;
    private final Servo topLight;
    private double estimatedAngle;
    private boolean atTarget;
    private double lastSensorVoltage;
    private double lastGoodVoltage;
    private double continuousVoltage;
    private final PIDController pid;
    private long lastTime;
    private final ElapsedTime timer;

    public Turret() {
        packet = new TelemetryPacket();
        turretEncoder = Robot.opMode.hardwareMap.get(AnalogInput.class, "TurretEncoder");
        turretSpin = Robot.opMode.hardwareMap.get(Servo.class, "TurretSpin");
        turretSpin.setDirection(Servo.Direction.REVERSE);
        topLight = Robot.opMode.hardwareMap.get(Servo.class, "RGB_VisionAcquired");
        atTarget = false;
        //start in middle of sensor range
        lastSensorVoltage = 1.5;
        continuousVoltage = lastSensorVoltage;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //cannot have I term, as we don't reset the pids in continous modes
        pid = new PIDController(Robot.RobotConfig.TURRET_KP, Robot.RobotConfig.TURRET_KI,Robot.RobotConfig.TURRET_KD, 0.05);
        resetPid();

        //zero the turret on startup
        double voltage = turretEncoder.getVoltage();
        estimatedAngle = -49.49*voltage - 73.04;
        turretSpin.setPosition(0.5);
    }

    @Override
    public void periodic() {
        double voltage = turretEncoder.getVoltage();

        //sometimes on rollover, we catch 3.2, then 1.4, then 0.1 during the transition of rollover,
        //this if check catches when we settle
        if(Math.abs(lastSensorVoltage - voltage) < 0.9) {
            //voltage hops from 0 to 3.22v, then back to zero
            if (lastGoodVoltage > 2.2 && voltage < 1.1) {
                RobotState.turretRollovers++;
            } else if (voltage > 2.2 && lastGoodVoltage < 1.1) {
                RobotState.turretRollovers--;
            }
            continuousVoltage = (3.296 * RobotState.turretRollovers) + voltage;
            lastGoodVoltage = voltage;
        }
        lastSensorVoltage = voltage;

        estimatedAngle = -49.49*continuousVoltage - 73.04;

        packet.put("Turret/EstimatedAngle", getAngle());
        packet.put("Turret/FeedbackVoltage", voltage);
        packet.put("Turret/LastSensorVoltage", lastSensorVoltage);
        packet.put("Turret/LastGoodVoltage", lastGoodVoltage);
        packet.put("Turret/Rollovers", RobotState.turretRollovers);
        packet.put("Turret/ContinuousVoltage", continuousVoltage);
        packet.put("Turret/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("Turret/AtTarget", atTarget);
        Robot.logPacket(packet);

        pid.m_kp = Robot.RobotConfig.TURRET_KP;
        pid.m_ki = Robot.RobotConfig.TURRET_KI;
        pid.m_kd = Robot.RobotConfig.TURRET_KD;
    }

    private void setAngle(double angleDeg) {
        if (!Robot.IsRed) {
            //if blue, adjust to make
            angleDeg +=4;
        } else {
            angleDeg -=4;
        }
        var clamp = MathUtil.clamp(angleDeg, -360., 0);
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1_000_000_000.;
        double deltaAngle = Math.abs(angleDeg - getAngle());
        boolean inRange = deltaAngle < 2;
        if(deltaAngle > 15 || deltaTime > 0.2) {
            resetPid();
        }

        var output = pid.calculate(getAngle(), clamp, deltaTime);
        if(!Double.isNaN(output)) {
            //the servo doesn't run between 0.47-0.53, so bump up the requests
            if(output < -0.001) {
                output -= 0.025;
            } else if (output > 0.001) {
                output += 0.025;
            } else {
                output = 0;
            }
            output *= -1;

            //make soft limits for requests
            if(output < 0 && getAngle() < -120) {
                //output = 0;
            } else if (output > 0 && getAngle() > 120) {
                //output = 0;
            }

            if(inRange) {
                output = 0;
            }
            var outClamp = MathUtil.clamp(output + 0.5, 0.30, 0.70);
            turretSpin.setPosition(outClamp);
            packet.put("Turret/OutClamp", outClamp);
            lastTime = currentTime;
        } else {
            pid.reset();
        }

        packet.put("Turret/RequestAngle", angleDeg);

        if(!inRange) {
            timer.reset();
        }
        atTarget = timer.time() > 200;
    }

    public void resetPid() {
        pid.reset();
        lastTime = System.nanoTime();
        turretSpin.setDirection(Servo.Direction.REVERSE);
        turretSpin.setPosition(0.5);
    }

    public void aimWithLimelight(double X_Error) {
        double GainFactor;
        double CorrectionNeeded;

        //X_Error is how many degrees off center the tag is
        if (Math.abs(X_Error) < 0.8) {
            GainFactor = 0;
        } else if (Math.abs(X_Error) < 10) {
            GainFactor = 0.8;
        } else {
            GainFactor = 1;
        }
        CorrectionNeeded = X_Error * Robot.RobotConfig.TURRET_CAMERA_AIM_P * GainFactor;

        //make soft limits for requests
        if(CorrectionNeeded > 0 && getAngle() < -360) {
            CorrectionNeeded = 0;
        } else if (CorrectionNeeded < 0 && getAngle() > 0) {
            CorrectionNeeded = 0;
        }

        var outClamp = MathUtil.clamp(0.5 - CorrectionNeeded, 0.30, 0.70);
        turretSpin.setPosition(outClamp);
        packet.put("Turret/OutClamp", outClamp);
    }

    public Command resetZero() {
        return new ResetZero();
    }

    public Command bumpRotations(int numRotations) {
        return new BumpRotations(numRotations);
    }

    public boolean atTarget() {
        return atTarget;
    }

    public double getAngle() {
        return estimatedAngle - Robot.RobotConfig.TURRET_OFFSET_DEG - RobotState.turretManualOffset;
    }

    public Command centerTurretViaPosition() {
        return new CenterTurretViaPosition();
    }

    public Command stopTurret() {
        return new StopTurret();
    }

    public Command preShotTurret(Pose shootingPose) {
        return new PreShotTurret(shootingPose);
    }

    public Command commandTurretAngle(double angleDeg) {
        return new CommandTurretAngle(angleDeg);
    }

    public Command bumpTurretHome(double angleDeg) {
        return new InstantCommand(
                () -> {
                    RobotState.turretManualOffset += angleDeg;}
        );
    }

    public void setLed(double color) {
        topLight.setPosition(color);
    }

    public Command setLedCommand(double color) {
        return new CommandBase() {
            @Override
            public void execute() {
                topLight.setPosition(color);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    private class CommandTurretAngle extends CommandBase {
        double angle;
        public CommandTurretAngle(double angle) {
            this.angle = angle;
            addRequirements(Robot.turret);
        }

        @Override
        public void initialize() {
            timer.reset();
        }

        @Override
        public void execute() {
            setAngle(angle);
        }

        @Override
        public boolean isFinished() {
            return atTarget;
        }

        @Override
        public void end(boolean interrupted) {
            turretSpin.setPosition(0.5);
        }
    }

    private class CenterTurretViaPosition extends CommandBase {
        public CenterTurretViaPosition() {
            addRequirements(Robot.turret);
        }

        @Override
        public void initialize() {
            timer.reset();
        }

        @Override
        public void execute() {
            var targetX = Robot.vision.getTargetX();

            if (targetX.isPresent()) {
                aimWithLimelight(targetX.get());
            } else {
                double angle = Robot.drivetrain.getGoalAngle() - Robot.drivetrain.getHeading();
                setAngle(angle);
            }
            packet.put("Turret/UsingTag", targetX.isPresent());
        }

        @Override
        public boolean isFinished() {
            return atTarget;
        }

        @Override
        public void end(boolean interrupted) {
            turretSpin.setPosition(0.5);
            packet.put("Turret/UsingTag", false);
        }
    }

    private class PreShotTurret extends CommandBase {
        Pose pose;
        double angle;
        public PreShotTurret(Pose shootingPose) {
            this.pose = shootingPose;
            addRequirements(Robot.turret);
        }

        @Override
        public void initialize() {
            angle = DrivetrainPP.getGoalTarget(pose).GoalAngle - (90 - Math.toDegrees(pose.getHeading()));
            timer.reset();
        }

        @Override
        public void execute() {
            angle = DrivetrainPP.getGoalTarget(pose).GoalAngle - (90 - Math.toDegrees(pose.getHeading()));
            setAngle(angle);
        }
    }

    private class ResetZero extends CommandBase {
        @Override
        public void execute() {
            RobotState.turretManualOffset = estimatedAngle;
            RobotState.turretRollovers = 0;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
    private class StopTurret extends CommandBase {
        @Override
        public void execute() {
            turretSpin.setPosition(0.5);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private class BumpRotations extends CommandBase {
        private final int numRotations;
        public BumpRotations(int numRotations) {
            this.numRotations = numRotations;
        }
        @Override
        public void execute() {
            RobotState.turretRollovers += numRotations;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public Command testTurret() {
        return new SequentialCommandGroup(
                new CommandTurretAngle(-90),
                new WaitCommand(3000),
                new CommandTurretAngle(-45),
                new WaitCommand(3000),
                new CommandTurretAngle(0),
                new WaitCommand(3000),
                new CommandTurretAngle(45),
                new WaitCommand(3000),
                new CommandTurretAngle(90),
                new WaitCommand(3000)
        );
    }
}
