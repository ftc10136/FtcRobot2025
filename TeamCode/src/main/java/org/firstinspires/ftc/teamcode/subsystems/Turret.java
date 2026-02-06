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
import org.livoniawarriors.RobotUtil;

public class Turret extends SubsystemBase {
    private final TelemetryPacket packet;
    private final AnalogInput turretEncoder;
    private final Servo turretSpin;
    private final Servo topLight;
    private double estimatedCommand;
    private double estimatedAngle;

    public Turret() {
        packet = new TelemetryPacket();
        turretEncoder = Robot.opMode.hardwareMap.get(AnalogInput.class, "TurretEncoder");
        turretSpin = Robot.opMode.hardwareMap.get(Servo.class, "TurretSpin");
        turretSpin.setDirection(Servo.Direction.FORWARD);
        topLight = Robot.opMode.hardwareMap.get(Servo.class, "RGB_VisionAcquired");
        //during HP load, move turret to 1, then reset back to started position

        /* Limelight Tracking
         *  CorrectionNeeded = X_Error * -0.005 * GainFactor;
            if (TurretMode == 1) {
                TurretSpin.setPosition(TurretSpin.getPosition() - CorrectionNeeded);
            }
         */
    }

    @Override
    public void periodic() {
        double voltage = turretEncoder.getVoltage();
        //these were calculated by running testTurret(), reading the voltage vs command values, and run a linear regression on the data
        estimatedCommand = -0.3377*voltage + 1.058;
        //these were calculated by moving the turret to straight into intake (0*), and to the empty side (90*), and doing a linear regression on it
        estimatedAngle = -53.92*voltage + 91.67;
        packet.put("Turret/ServoCommand", turretSpin.getPosition());
        packet.put("Turret/EstimatedCommand", estimatedCommand);
        packet.put("Turret/EstimatedAngle", estimatedAngle);
        packet.put("Turret/FeedbackVoltage", voltage);
        packet.put("Turret/Command", RobotUtil.getCommandName(getCurrentCommand()));
        Robot.logPacket(packet);
        Robot.opMode.telemetry.addData("TurretFeedback", turretEncoder.getVoltage());
    }

    private void setPosition(double servoPos) {
        turretSpin.setPosition(servoPos);
    }

    private void setAngle(double angleDeg) {
        //inverting the voltage to angle formulas
        double voltage = (angleDeg - 91.67)/-53.93;
        double command = -0.3377*voltage + 1.058;
        setPosition(command);
    }

    public double getAngle() {
        return estimatedAngle;
    }

    public Command centerTurretViaVision() {
        return new CenterTurretViaVision();
    }

    public Command commandTurretAngle(double angleDeg) {
        return new CommandTurretAngle(angleDeg);
    }

    public Command setLed() {
        return new CommandBase() {
            @Override
            public void execute() {
                topLight.setPosition(0.5);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    private class CommandTurret extends CommandBase {
        double pos;
        public CommandTurret(double pos) {
            this.pos = pos;
            addRequirements(Robot.turret);
        }

        @Override
        public void execute() {
            setPosition(pos);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(pos - estimatedCommand) < 0.01;
        }
    }

    private class CommandTurretAngle extends CommandBase {
        double angle;
        public CommandTurretAngle(double angle) {
            this.angle = angle;
            addRequirements(Robot.turret);
        }

        @Override
        public void execute() {
            setAngle(angle);
        }

        @Override
        public boolean isFinished() {
            //in degrees
            return Math.abs(angle - getAngle()) < 3;
        }
    }

    private class CenterTurretViaVision extends CommandBase {
        boolean finished = false;
        public CenterTurretViaVision() {
            addRequirements(Robot.turret);
        }

        @Override
        public void initialize() {
            finished = false;
        }

        @Override
        public void execute() {
            double GainFactor;
            double X_Error = Robot.vision.getTurretError();
            if (Math.abs(X_Error) < 0.8) {
                GainFactor = 0;
                finished = true;
            } else if (Math.abs(X_Error) < 10) {
                GainFactor = 0.8;
            } else {
                GainFactor = 1;
            }
            double CorrectionNeeded = X_Error * -0.005 * GainFactor;
            setPosition(turretSpin.getPosition() - CorrectionNeeded);
        }

        @Override
        public boolean isFinished() {
            return finished;
        }
    }

    public Command testTurret() {
        return new SequentialCommandGroup(
                new CommandTurret(0.3),
                new WaitCommand(3000),
                new CommandTurret(0.4),
                new WaitCommand(3000),
                new CommandTurret(0.5),
                new WaitCommand(3000),
                new CommandTurret(0.6),
                new WaitCommand(3000),
                new CommandTurret(0.7),
                new WaitCommand(3000)
        );
    }
}
