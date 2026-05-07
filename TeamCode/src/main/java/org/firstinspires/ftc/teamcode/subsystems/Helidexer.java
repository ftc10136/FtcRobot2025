package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotState;
import org.livoniawarriors.GoBildaLedColors;
import org.livoniawarriors.RobotUtil;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;

public class Helidexer extends SubsystemBase {
    private final DcMotorEx helixMotor;
    private final TelemetryPacket packet;
    private double motorCurrent;
    private final HashMap<SpinBay.BayState, Double> bayStateToLedCommands;
    private final HashMap<String, SpinBay> bays;
    private int rawPosition;
    private DigitalChannel greenIntake;
    private DigitalChannel purpleIntake;

    private int loops = 0;
    private int priorityBay = 1;

    public Helidexer() {
        packet = new TelemetryPacket();
        helixMotor = Robot.opMode.hardwareMap.get(DcMotorEx.class, "Helidexer");
        //Purple/Green
        purpleIntake = Robot.opMode.hardwareMap.get(DigitalChannel.class, "Purple");
        greenIntake = Robot.opMode.hardwareMap.get(DigitalChannel.class, "Green");
        helixMotor.setDirection(DcMotor.Direction.REVERSE);
        helixMotor.setTargetPositionTolerance((int)(Robot.RobotConfig.POSITION_TOLERANCE * 0.8));
        rawPosition = helixMotor.getCurrentPosition();
        helixMotor.setTargetPosition(RobotState.helidexerOffset);
        helixMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        helixMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        rawPosition = helixMotor.getCurrentPosition();
        loops++;
        int currentBay = (getCurrentBay() % 3) + 1;
        priorityBay = (loops % 3) + 1;
        /*
        if (Robot.intake.isRunning() && getBayState(currentBay) == SpinBay.BayState.None) {
            //if we are intaking a bay, see if it has a ball ASAP

        } else {
            //default to checking each bay once a loop
            priorityBay = (loops % 3) + 1;
            for(int i=1; i<=3; i++) {
                //if a bay is something, it gets more priority to find a color
                if (getBayState(i) == SpinBay.BayState.Something) {
                    priorityBay = i;
                    break;
                }
            }
        }*/
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
        packet.put("Helidexer/Position", rawPosition);
        packet.put("Helidexer/CurrentBay", getCurrentBay());
        packet.put("Helidexer/IntakePurple", purpleIntake.getState());
        packet.put("Helidexer/IntakeGreen", greenIntake.getState());
        packet.put("Currents/Helidexer", helixMotor.getCurrent(CurrentUnit.AMPS));
        Robot.logPacket(packet);
    }

    public double getLedColor(SpinBay.BayState state) {
        //noinspection DataFlowIssue
        return bayStateToLedCommands.get(state);
    }

    public int getPriorityBay() {
        return priorityBay;
    }

    public SpinBay.BayState getBayState(int bay) {
        return Objects.requireNonNull(bays.get("Bay" + bay)).getState();
    }

    public void setBayState(int bay, SpinBay.BayState state) {
        Objects.requireNonNull(bays.get("Bay" + bay)).setState(state);
    }

    public Command bumpHelidexer(int steps) {
        return new InstantCommand(()->{RobotState.helidexerManualOffset += steps;});
    }

    public BooleanSupplier hasBall(int bay) {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getBayState(bay) != SpinBay.BayState.None;
            }
        };
    }

    /// Get how many counts to drive to a position at that bay, can be >3
    private int getBayPos(int bay) {
        return RobotState.helidexerOffset + (bay * Robot.RobotConfig.COUNTS_PER_BAY) + RobotState.helidexerManualOffset;
    }

    /// Based on counts, what bay have we currently rotated to
    private int getCurrentBay() {
        double estBay = (double)(helixMotor.getCurrentPosition() - RobotState.helidexerOffset - RobotState.helidexerManualOffset) / Robot.RobotConfig.COUNTS_PER_BAY;
        return Math.toIntExact(Math.round(estBay));
    }

    public int getRawPosition() {
        return rawPosition;
    }

    public boolean isMotifPossible() {
        int purple = 0;
        int green = 0;

        for (int i=1; i<=3; i++) {
            var state = getBayState(i);
            if (state == SpinBay.BayState.Green) {
                green++;
            } else if (state == SpinBay.BayState.Purple) {
                purple++;
            }
        }
        return (purple == 2) && (green == 1);
    }

    public void resetBayStates() {
        Objects.requireNonNull(bays.get("Bay1")).resetBayState();
        Objects.requireNonNull(bays.get("Bay2")).resetBayState();
        Objects.requireNonNull(bays.get("Bay3")).resetBayState();
    }

    public Command advanceBay() {
        return new AdvanceBay();
    }

    public Command shootAll() {
        return new ShootAll();
    }

    public Command primeForShot() {
        return new PrimeForShot();
    }

    public Command primeForMotif() {
        return new ConditionalCommand(new PrimeForMotif(), primeForShot(), this::isMotifPossible);
    }

    public Command commandFloorLoadUntilBall(int bay) {
        return new CommandFloorLoadUntilBall(bay);
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
            pos = getBayPos(getCurrentBay()+1);
            helixMotor.setTargetPosition(pos);
            helixMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
            overcurrent = false;
        }

        @Override
        public void execute() {
            pos = getBayPos(getCurrentBay()+1);
            helixMotor.setTargetPosition(pos);
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

        @Override
        public void end(boolean interrupted) {
            helixMotor.setPower(0);
        }
    }

    private class ShootAll extends CommandBase {
        int pos;

        public ShootAll() {
            addRequirements(Robot.helidexer);
        }

        @Override
        public void execute() {
            pos = getBayPos(0);
            helixMotor.setTargetPosition(pos);
            helixMotor.setPower(-Robot.RobotConfig.HELIDEXER_P);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(helixMotor.getCurrentPosition() - pos) < Robot.RobotConfig.POSITION_TOLERANCE;
        }

        @Override
        public void end(boolean interrupted) {
            resetBayStates();
            helixMotor.setPower(0);
        }
    }

    private class PrimeForShot extends CommandBase {
        int pos;
        public PrimeForShot() {
            addRequirements(Robot.helidexer);
        }

        @Override
        public void initialize() {
            int minBays;
            //check if we have gone enough bays to shoot
            if(Objects.requireNonNull(bays.get("Bay3")).getState() != SpinBay.BayState.None) {
                minBays = 3;
            } else if (Objects.requireNonNull(bays.get("Bay2")).getState() != SpinBay.BayState.None) {
                minBays = 2;
            } else if (Objects.requireNonNull(bays.get("Bay1")).getState() != SpinBay.BayState.None) {
                minBays = 1;
            } else {
                minBays = 0;
            }

            int minPos = getBayPos(minBays);
            if(helixMotor.getCurrentPosition() > minPos) {
                //we are in position, no need to change
                pos = helixMotor.getCurrentPosition();
            } else {
                pos = minPos;
                helixMotor.setTargetPosition(pos);
                helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
            }

        }

        @Override
        public void execute() {
            helixMotor.setTargetPosition(pos);
            helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(helixMotor.getCurrentPosition() - pos) < Robot.RobotConfig.POSITION_TOLERANCE;
        }

        @Override
        public void end(boolean interrupted) {
            helixMotor.setPower(0);
        }
    }

    public class CommandFloorLoadUntilBall extends CommandBase {
        int targetBay;
        int bay;
        int pos;
        int outBay;
        Timer finished;
        public CommandFloorLoadUntilBall(int bay) {
            this.bay = bay;
            targetBay = bay-1;
            finished = new Timer();
            addRequirements(Robot.helidexer);
        }

        @Override
        public void initialize() {
            int currentBay = getCurrentBay();
            int curRotations = currentBay / 3;
            int curBay = currentBay % 3;

            if (curBay == targetBay) {
                outBay = currentBay;
            } else if (curBay > targetBay) {
                outBay = (curRotations + 1) * 3 + targetBay;
            } else {
                outBay = (curRotations) * 3 + targetBay;
            }
            pos = getBayPos(outBay);
            helixMotor.setTargetPosition(pos);
            helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
            finished.resetTimer();

        }

        @Override
        public void execute() {
            helixMotor.setTargetPosition(pos);
            helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
            boolean highColorSensor = purpleIntake.getState() || greenIntake.getState();
            boolean bayMatches = outBay == getCurrentBay();
            if (bayMatches && highColorSensor) {
                setBayState(bay, SpinBay.BayState.Something);
            } else {
                finished.resetTimer();
            }
        }

        @Override
        public boolean isFinished() {
            return finished.getElapsedTimeSeconds() > 0.25 || (getBayState(bay) == SpinBay.BayState.Purple || getBayState(bay) == SpinBay.BayState.Green);
        }

        @Override
        public void end(boolean interrupted) {
            helixMotor.setPower(0);
        }
    }


    private class PrimeForMotif extends CommandBase {
        int pos;
        public PrimeForMotif() {
            addRequirements(Robot.helidexer);
        }

        @Override
        public void execute() {
            pos = getBayPos(getTargetBay());
            helixMotor.setTargetPosition(pos);
            helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
        }

        private int getTargetBay() {
            Vision.Motifs motif = Robot.vision.getSeenMotif();

            int currentBay = getCurrentBay();
            int targetGreenBay;
            if(motif == Vision.Motifs.GPP) {
                targetGreenBay = 1;
            } else if (motif == Vision.Motifs.PGP) {
                targetGreenBay = 2;
            } else {
                //Unknown or PPG
                targetGreenBay = 0;
            }

            int currentGreenBay = 1;
            for(int i = 1; i<=3; i++) {
                if(getBayState(i) == SpinBay.BayState.Green) {
                    int modBay = currentBay % 3;
                    if(i == 1 && modBay == 0) currentGreenBay = 0;
                    if(i == 1 && modBay == 1) currentGreenBay = 1;
                    if(i == 1 && modBay == 2) currentGreenBay = 2;
                    if(i == 2 && modBay == 0) currentGreenBay = 2;
                    if(i == 2 && modBay == 1) currentGreenBay = 0;
                    if(i == 2 && modBay == 2) currentGreenBay = 1;
                    if(i == 3 && modBay == 0) currentGreenBay = 1;
                    if(i == 3 && modBay == 1) currentGreenBay = 2;
                    if(i == 3 && modBay == 2) currentGreenBay = 0;
                }
            }

            //steven hates this, but the formula is too hard and it's too late
            int targetBay=0;
            if(targetGreenBay == 0 && currentGreenBay == 0) targetBay = 0;
            if(targetGreenBay == 0 && currentGreenBay == 1) targetBay = 2;
            if(targetGreenBay == 0 && currentGreenBay == 2) targetBay = 1;
            if(targetGreenBay == 1 && currentGreenBay == 0) targetBay = 1;
            if(targetGreenBay == 1 && currentGreenBay == 1) targetBay = 0;
            if(targetGreenBay == 1 && currentGreenBay == 2) targetBay = 2;
            if(targetGreenBay == 2 && currentGreenBay == 0) targetBay = 2;
            if(targetGreenBay == 2 && currentGreenBay == 1) targetBay = 1;
            if(targetGreenBay == 2 && currentGreenBay == 2) targetBay = 0;

            int outBay = currentBay + targetBay;
            if(outBay < 3) {
                outBay += 3;
            }

            packet.put("Helidexer/targetGreenBay", targetGreenBay);
            packet.put("Helidexer/currentGreenBay", currentGreenBay);
            packet.put("Helidexer/targetBay", targetBay);
            packet.put("Helidexer/outBay", outBay);
            return outBay;
        }

        @Override
        public boolean isFinished() {
            return Math.abs(helixMotor.getCurrentPosition() - pos) < Robot.RobotConfig.POSITION_TOLERANCE;
        }

        @Override
        public void end(boolean interrupted) {
            helixMotor.setPower(0);
        }
    }
}
