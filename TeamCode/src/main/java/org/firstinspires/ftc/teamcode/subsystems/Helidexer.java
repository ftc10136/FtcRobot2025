package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.revrobotics.ColorMatch;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.GoBildaLedColors;
import org.livoniawarriors.RobotUtil;

import java.util.HashMap;

import edu.wpi.first.wpilibj.util.Color;

public class Helidexer extends SubsystemBase {
    private final DcMotorEx helixMotor;
    private final TelemetryPacket packet;
    private int sensorHome;
    private int currentBay;
    private double motorCurrent;
    private final ColorMatch matcher;
    private final HashMap<SpinBay.BayState, Double> bayStateToLedCommands;
    private final HashMap<Color, SpinBay.BayState> colorToBayState;
    private final HashMap<String, SpinBay> bays;

    public Helidexer() {
        packet = new TelemetryPacket();
        helixMotor = Robot.opMode.hardwareMap.get(DcMotorEx.class, "Helidexer");
        helixMotor.setDirection(DcMotor.Direction.REVERSE);
        helixMotor.setTargetPositionTolerance((int)(Robot.RobotConfig.POSITION_TOLERANCE * 0.8));
        sensorHome = helixMotor.getCurrentPosition();
        helixMotor.setTargetPosition(sensorHome);
        helixMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        helixMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        currentBay = 0;

        Color purpleInput = new Color(0.571, 0.654, 0.971);
        Color greenInput = new Color(0.303, 1., 0.753);

        matcher = new ColorMatch();
        matcher.setConfidenceThreshold(0.65);
        matcher.addColorMatch(purpleInput);
        matcher.addColorMatch(greenInput);

        colorToBayState = new HashMap<>();
        colorToBayState.put(purpleInput, SpinBay.BayState.Purple);
        colorToBayState.put(greenInput, SpinBay.BayState.Green);
        colorToBayState.put(Color.kOrange, SpinBay.BayState.Something);
        colorToBayState.put(Color.kBlack, SpinBay.BayState.None);

        bayStateToLedCommands = new HashMap<>();
        bayStateToLedCommands.put(SpinBay.BayState.None, GoBildaLedColors.Off);
        bayStateToLedCommands.put(SpinBay.BayState.Something, GoBildaLedColors.Orange);
        bayStateToLedCommands.put(SpinBay.BayState.Green, GoBildaLedColors.Green);
        bayStateToLedCommands.put(SpinBay.BayState.Purple, GoBildaLedColors.Purple);

        bays = new HashMap<>();
        bays.put("Bay1", new SpinBay("Bay1A-Color", "RGB-Bay1", 1));
        bays.put("Bay2", new SpinBay("Bay2A-Color", "RGB-Bay2", 2));
        bays.put("Bay3", new SpinBay("Bay3A-Color", "RGB-Bay3", 3));
    }

    @Override
    public void periodic() {
        for (var entry : bays.entrySet()) {
            var key = entry.getKey();
            var bay = entry.getValue();
            bay.periodic();
            Color color = bay.getColor();
            packet.put("Helidexer/" + key + "/Red", color.red);
            packet.put("Helidexer/" + key + "/Green", color.green);
            packet.put("Helidexer/" + key + "/Blue", color.blue);
            packet.put("Helidexer/" + key + "/Distance", bay.getDist());
            packet.put("Helidexer/" + key + "/State", bay.getState());
            packet.put("Helidexer/" + key + "/ReadTime", bay.reading.loopTimeMs);
        }
        motorCurrent = helixMotor.getCurrent(CurrentUnit.AMPS);
        packet.put("Helidexer/Power", helixMotor.getPower());
        packet.put("Helidexer/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("Helidexer/Position", helixMotor.getCurrentPosition());
        packet.put("Helidexer/CurrentBay", currentBay);
        packet.put("Currents/Helidexer", helixMotor.getCurrent(CurrentUnit.AMPS));
        Robot.logPacket(packet);
    }

    public double getLedColor(SpinBay.BayState state) {
        //noinspection DataFlowIssue
        return bayStateToLedCommands.get(state);
    }

    public void resetHome() {
        sensorHome = helixMotor.getCurrentPosition();
    }

    public Command resetHomeCommand() {
        return new CommandBase() {
            @Override
            public void execute() {
                resetHome();
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public Command advanceBay() {
        return new AdvanceBay();
    }

    public Command shootAll() {
        return new ShootAll();
    }

    private class AdvanceBay extends CommandBase {
        int pos;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean overcurrent;

        public AdvanceBay() {
            addRequirements(Robot.helidexer);
        }

        @Override
        public void initialize() {
            currentBay++;
            pos = sensorHome + (currentBay * Robot.RobotConfig.COUNTS_PER_BAY);
            helixMotor.setTargetPosition(pos);
            helixMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
            overcurrent = false;
        }

        @Override
        public void execute() {
            if (overcurrent) {
                if(timer.milliseconds() > 300) {
                    overcurrent = false;
                    helixMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
                } else {
                    helixMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    helixMotor.setPower(-0.5);
                }
            } else {
                if (motorCurrent > 6) {
                    overcurrent = true;
                    timer.reset();
                    helixMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    helixMotor.setPower(-0.5);
                } else {
                    helixMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
                }
            }
        }

        @Override
        public boolean isFinished() {
            return Math.abs(helixMotor.getCurrentPosition() - pos) < Robot.RobotConfig.POSITION_TOLERANCE;
        }
    }

    private class ShootAll extends CommandBase {
        int pos;

        public ShootAll() {
            addRequirements(Robot.helidexer);
        }

        @Override
        public void initialize() {
            currentBay = 0;
            pos = sensorHome;
            helixMotor.setTargetPosition(pos);
            helixMotor.setPower(-Robot.RobotConfig.HELIDEXER_P);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(helixMotor.getCurrentPosition() - pos) < Robot.RobotConfig.POSITION_TOLERANCE;
        }
    }
}
