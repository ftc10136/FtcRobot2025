package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.revrobotics.ColorMatch;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.RobotUtil;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;

public class Spindexer extends SubsystemBase {

    private final Servo Spindexer;
    private final AnalogInput SpindexerEncoder;
    private final TelemetryPacket packet;
    private final ColorMatch matcher;

    public enum BayState {
        None,
        Something,
        Green,
        Purple
    }

    public enum SpindexerType {
        @SuppressWarnings("unused")
        Shoot,
        HumanIntake,
        FloorIntake
    }
    private final HashMap<BayState, Double> bayStateToLedCommands;
    private final HashMap<Color, BayState> colorToBayState;
    private final HashMap<String, SpinBay> bays;
    private double feedbackPos;

    public Spindexer() {
        super();
        Color purpleInput = new Color(0.571, 0.654, 0.971);
        Color greenInput = new Color(0.303, 1., 0.753);
        packet = new TelemetryPacket();

        matcher = new ColorMatch();
        matcher.setConfidenceThreshold(0.7);
        matcher.addColorMatch(purpleInput);
        matcher.addColorMatch(greenInput);

        colorToBayState = new HashMap<>();
        colorToBayState.put(purpleInput, BayState.Purple);
        colorToBayState.put(greenInput, BayState.Green);
        colorToBayState.put(Color.kOrange, BayState.Something);
        colorToBayState.put(Color.kBlack, BayState.None);

        bayStateToLedCommands = new HashMap<>();
        bayStateToLedCommands.put(BayState.None, 0.);
        bayStateToLedCommands.put(BayState.Something, 0.333);
        bayStateToLedCommands.put(BayState.Green, 0.5);
        bayStateToLedCommands.put(BayState.Purple, 0.722);

        bays = new HashMap<>();
        bays.put("Bay1", new SpinBay("Bay1A-Color", "RGB-Bay1"));
        bays.put("Bay2", new SpinBay("Bay2A-Color", "RGB-Bay2"));
        bays.put("Bay3", new SpinBay("Bay3A-Color", "RGB-Bay3"));

        Spindexer = Robot.opMode.hardwareMap.get(Servo.class, "Spindexer");
        Spindexer.setDirection(Servo.Direction.REVERSE);
        SpindexerEncoder = Robot.opMode.hardwareMap.get(AnalogInput.class, "SpindexerEncoder");
    }

    @Override
    public void periodic() {
        double voltage = SpindexerEncoder.getVoltage();
        //these numbers were calculated with a best fit approximation from the 3 bays
        feedbackPos = 0.3391 * voltage - 0.07272;
        for (var bay : bays.entrySet()) {
            bay.getValue().periodic();
            Color color = bay.getValue().getColor();
            packet.put("Spindexer/" + bay.getKey() + "/Red", color.red);
            packet.put("Spindexer/" + bay.getKey() + "/Green", color.green);
            packet.put("Spindexer/" + bay.getKey() + "/Blue", color.blue);
            packet.put("Spindexer/" + bay.getKey() + "/Distance", bay.getValue().getDist());
            packet.put("Spindexer/" + bay.getKey() + "/State", bay.getValue().getState());
        }
        packet.put("Spindexer/PositionServo", Spindexer.getPosition());
        packet.put("Spindexer/FeedbackVoltage", voltage);
        packet.put("Spindexer/PositionFeedback", feedbackPos);
        packet.put("Spindexer/Command", RobotUtil.getCommandName(getCurrentCommand()));
        Robot.logPacket(packet);
    }

    public BayState matchColor(Color color) {
        BayState state;
        var match = matcher.matchColor(color);
        if(match == null) {
            state = BayState.Something;
        } else {
            state = colorToBayState.get(match.color);
        }
        return state;
    }

    public double getLedColor(BayState state) {
        //using https://www.gobilda.com/rgb-indicator-light-pwm-controlled/

        //noinspection DataFlowIssue
        return bayStateToLedCommands.get(state);
    }

    private void setSpindexerPos(double pos) {
        Spindexer.setPosition(pos);
    }

    @SuppressWarnings("unused")
    public Command commandSpindexerPos(double pos) {
        return new SetSpindexerPos(pos);
    }

    public Command commandSpindexerPos(int bay, SpindexerType type) {
        return new SetSpindexerPos(bay, type);
    }

    public Command commandHpLoadUntilBall(int bay) {
        //using conditional command to speed up logic, if true, do nothing (instant command), otherwise
        //run the indexer until we see a ball
        return new ConditionalCommand(new InstantCommand(), commandSpindexerPos(bay, SpindexerType.HumanIntake).perpetually().interruptOn(hasBall(bay)), hasBall(bay));
    }

    public Command commandFloorLoadUntilBall(int bay) {
        //using conditional command to speed up logic, if true, do nothing (instant command), otherwise
        //run the indexer until we see a ball
        return new ConditionalCommand(new InstantCommand(), commandSpindexerPos(bay, SpindexerType.FloorIntake).perpetually().interruptOn(hasBall(bay)), hasBall(bay));
    }

    public BooleanSupplier hasBall(int bay) {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return Objects.requireNonNull(bays.get("Bay" + bay)).getState() != BayState.None;
            }
        };
    }

    @SuppressWarnings("unused")
    public static double getShootIndexPos(int bay) {
        double rawPos = Robot.RobotConfig.SPINDEXER_OFFSET + ((bay-1) * 0.188);
        return RobotUtil.inputModulus(rawPos, 0,1);
    }

    public static double getIndexPos(int bay, SpindexerType type) {
        double rawPos = Robot.RobotConfig.SPINDEXER_OFFSET + ((bay-1) * 0.188);
        if(type == SpindexerType.HumanIntake) {
            rawPos = rawPos - 0.142;
        } else if (type == SpindexerType.FloorIntake) {
            rawPos = rawPos + 0.11248;
        }
        //one rotation is exactly 0.565 units of servo (0.565/2=0.2825), so we want to keep within that circle
        return RobotUtil.inputModulus(rawPos, Robot.RobotConfig.SPINDEXER_OFFSET - 0.2825,Robot.RobotConfig.SPINDEXER_OFFSET + 0.2825);
    }

    private class SetSpindexerPos extends CommandBase {
        private final double pos;
        public SetSpindexerPos(double position) {
            pos = position;
            addRequirements(Robot.spindexer);
        }
        public SetSpindexerPos(int bay, SpindexerType locType) {
            pos = getIndexPos(bay, locType);
            addRequirements(Robot.spindexer);
        }
        @Override
        public void execute() {
            setSpindexerPos(pos);
        }
        @Override
        public boolean isFinished() {
            return Math.abs(pos - feedbackPos) < 0.005;
        }
    }
}
