package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.GoBildaLedColors;
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
    private boolean visionAiming;
    private double lastSensorVoltage;
    private double lastGoodVoltage;
    private double continuousVoltage;
    private int rollovers;
    private final PIDController pid;
    private long lastTime;
    private double offsetAngle;

    public Turret() {
        packet = new TelemetryPacket();
        turretEncoder = Robot.opMode.hardwareMap.get(AnalogInput.class, "TurretEncoder");
        turretSpin = Robot.opMode.hardwareMap.get(Servo.class, "TurretSpin");
        turretSpin.setDirection(Servo.Direction.FORWARD);
        topLight = Robot.opMode.hardwareMap.get(Servo.class, "RGB_VisionAcquired");
        atTarget = false;
        visionAiming = false;
        //start in middle of sensor range
        lastSensorVoltage = 1.5;
        continuousVoltage = lastSensorVoltage;
        offsetAngle = 0;
        pid = new PIDController(0.002, 0.0005,0, 0.05);
        resetPid();

        //zero the turret on startup
        double voltage = turretEncoder.getVoltage();
        estimatedAngle = -38.58*voltage + 70.96;
        offsetAngle = estimatedAngle;
    }

    @Override
    public void periodic() {
        double voltage = turretEncoder.getVoltage();

        //sometimes on rollover, we catch 3.2, then 1.4, then 0.1 during the transition of rollover,
        //this if check catches when we settle
        if(Math.abs(lastSensorVoltage - voltage) < 0.5) {
            //voltage hops from 0 to 3.22v, then back to zero
            if (lastGoodVoltage > 2.7 && voltage < 0.5) {
                rollovers++;
            } else if (voltage > 2.7 && lastGoodVoltage < 0.5) {
                rollovers--;
            }
            continuousVoltage = (3.22 * rollovers) + voltage;
            lastGoodVoltage = voltage;
        }
        lastSensorVoltage = voltage;

        estimatedAngle = -38.58*continuousVoltage + 70.96;

        packet.put("Turret/ServoCommand", turretSpin.getPosition());
        packet.put("Turret/EstimatedAngle", getAngle());
        packet.put("Turret/FeedbackVoltage", voltage);
        packet.put("Turret/ContinuousVoltage", continuousVoltage);
        packet.put("Turret/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("Turret/AtTarget", atTarget);
        Robot.logPacket(packet);
        Robot.opMode.telemetry.addData("TurretFeedback", turretEncoder.getVoltage());
    }

    private void setAngle(double angleDeg) {
        var clamp = MathUtil.clamp(angleDeg, -60., 300.);
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1_000_000_000.;

        var output = pid.calculate(getAngle(), clamp, deltaTime);
        if(!Double.isNaN(output)) {
            if(output < 0 && getAngle() < -60) {
                output = 0;
            } else if (output > 0 && getAngle() > 300) {
                output = 0;
            }
            var outClamp = MathUtil.clamp(output + 0.5, 0.4, 0.6);
            turretSpin.setPosition(outClamp);
            packet.put("Turret/OutClamp", outClamp);
            lastTime = currentTime;
        } else {
            pid.reset();
        }

        packet.put("Turret/DeltaTime", deltaTime);
        Robot.opMode.telemetry.addData("LoopTime", deltaTime);
        packet.put("Turret/RequestAngle", angleDeg);
        atTarget = Math.abs(angleDeg - getAngle()) < 3;
    }

    private void resetPid() {
        pid.reset();
        lastTime = System.nanoTime();
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
        return estimatedAngle - offsetAngle;
    }

    public Command centerTurretViaPosition() {
        return new CenterTurretViaPosition();
    }

    public Command commandTurretAngle(double angleDeg) {
        return new CommandTurretAngle(angleDeg);
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
            resetPid();
        }

        @Override
        public void execute() {
            setAngle(angle);
        }

        @Override
        public boolean isFinished() {
            return atTarget;
        }
    }

    private class CenterTurretViaPosition extends CommandBase {
        public CenterTurretViaPosition() {
            addRequirements(Robot.turret);
            resetPid();
        }

        @Override
        public void execute() {
            double angle = Robot.drivetrain.getGoalAngle() - Robot.drivetrain.getHeading();
            setAngle(angle);
        }

        @Override
        public boolean isFinished() {
            return atTarget;
        }
    }

    private class ResetZero extends CommandBase {
        @Override
        public void execute() {
            offsetAngle = estimatedAngle;
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
            rollovers += numRotations;
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
