package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.RobotUtil;

public class Helidexer extends SubsystemBase {
    private final DcMotorEx helixMotor;
    private final TelemetryPacket packet;
    private int sensorHome;
    private int currentBay;

    public Helidexer() {
        packet = new TelemetryPacket();
        helixMotor = Robot.opMode.hardwareMap.get(DcMotorEx.class, "Helidexer");
        helixMotor.setDirection(DcMotor.Direction.FORWARD);
        helixMotor.setTargetPositionTolerance((int)(Robot.RobotConfig.POSITION_TOLERANCE * 0.8));
        helixMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        helixMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sensorHome = helixMotor.getCurrentPosition();
        currentBay = 0;
    }

    @Override
    public void periodic() {
        packet.put("Helidexer/Power", helixMotor.getPower());
        packet.put("Helidexer/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("Helidexer/Position", helixMotor.getCurrentPosition());
        packet.put("Helidexer/CurrentBay", currentBay);
        packet.put("Currents/Helidexer", helixMotor.getCurrent(CurrentUnit.AMPS));
        Robot.logPacket(packet);
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

        public AdvanceBay() {
            addRequirements(Robot.helidexer);
        }

        @Override
        public void initialize() {
            currentBay++;
            pos = sensorHome + (currentBay * Robot.RobotConfig.COUNTS_PER_BAY);
            helixMotor.setTargetPosition(pos);
            helixMotor.setPower(Robot.RobotConfig.HELIDEXER_P);
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
