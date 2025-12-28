package org.firstinspires.ftc.teamcode.Ballevator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotUtil;

public class Ballevator extends SubsystemBase {
    private final TelemetryPacket packet;
    private final AnalogInput BallevatorEncoder;
    private final Servo Ballevator;

    @SuppressWarnings("FieldCanBeLocal")
    private final double BALLEVATOR_DOWN_VOLTAGE = 2.9;
    @SuppressWarnings("FieldCanBeLocal")
    private final double BALLEVATOR_DOWN_COMMAND = 0;
    @SuppressWarnings("FieldCanBeLocal")
    private final double BALLEVATOR_UP_VOLTAGE = 2.297;
    @SuppressWarnings("FieldCanBeLocal")
    private final double BALLEVATOR_UP_COMMAND = 0.25;

    public Ballevator() {
        packet = new TelemetryPacket();
        Ballevator = Robot.opMode.hardwareMap.get(Servo.class, "Ballevator");
        BallevatorEncoder = Robot.opMode.hardwareMap.get(AnalogInput.class, "BallevatorEncoder");
        Ballevator.setPosition(0);
        Ballevator.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void periodic() {
        packet.put("Ballevator/ServoCommand", Ballevator.getPosition());
        packet.put("Ballevator/FeedbackVoltage", BallevatorEncoder.getVoltage());
        packet.put("Ballevator/Command", RobotUtil.getCommandName(getCurrentCommand()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public Command commandUp() {
        return new BallevatorUp();
    }

    public Command commandDown() {
        return new BallevatorDown();
    }

    public boolean isBallevatorDown() {
        return BallevatorEncoder.getVoltage() > BALLEVATOR_DOWN_VOLTAGE;
    }

    @SuppressWarnings("unused")
    public Trigger ballevatorDown() {
        return new Trigger() {
            @Override
            public boolean get() {
                return isBallevatorDown();
            }
        };
    }

    private class BallevatorUp extends CommandBase {
        public BallevatorUp() {
            addRequirements(Robot.intake);
        }
        @Override
        public void execute() {
            Ballevator.setPosition(BALLEVATOR_UP_COMMAND);
        }
        @Override
        public boolean isFinished() {
            return BallevatorEncoder.getVoltage() < BALLEVATOR_UP_VOLTAGE;
        }
    }

    private class BallevatorDown extends CommandBase {
        public BallevatorDown() {
            addRequirements(Robot.intake);
        }
        @Override
        public void execute() {
            Ballevator.setPosition(BALLEVATOR_DOWN_COMMAND);
        }
        @Override
        public boolean isFinished() {
            return isBallevatorDown();
        }
    }

    @SuppressWarnings("unused")
    public Command testBallevator() {
        return new RepeatCommand(new SequentialCommandGroup(
                commandUp(),
                new WaitCommand(3000),
                commandDown(),
                new WaitCommand(3000)
        ));
    }
}
