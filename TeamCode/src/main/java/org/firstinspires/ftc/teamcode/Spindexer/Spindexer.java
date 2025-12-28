package org.firstinspires.ftc.teamcode.Spindexer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.revrobotics.ColorMatch;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.HashMap;

import edu.wpi.first.wpilibj.util.Color;

public class Spindexer extends SubsystemBase {

    private final TelemetryPacket packet;
    private final ColorMatch matcher;

    public enum BayState {
        None,
        Something,
        Green,
        Purple
    }
    private final HashMap<BayState, Double> bayStateToLedCommands;
    private final HashMap<Color, BayState> colorToBayState;
    private final HashMap<String, SpinBay> bays;
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
    }

    @Override
    public void periodic() {
        for (var bay : bays.entrySet()) {
            bay.getValue().periodic();
            Color color = bay.getValue().getColor();
            packet.put("Spindexer/" + bay.getKey() + "/Red", color.red);
            packet.put("Spindexer/" + bay.getKey() + "/Green", color.green);
            packet.put("Spindexer/" + bay.getKey() + "/Blue", color.blue);
            packet.put("Spindexer/" + bay.getKey() + "/Distance", bay.getValue().getDist());
            packet.put("Spindexer/" + bay.getKey() + "/State", bay.getValue().getState());
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
}
