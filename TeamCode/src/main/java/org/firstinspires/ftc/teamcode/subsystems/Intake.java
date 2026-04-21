package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.RobotUtil;

public class Intake extends SubsystemBase {
    private final DcMotorEx Intake_Motor;
    private final TelemetryPacket packet;
    private double lastPower;

    public Intake() {
        packet = new TelemetryPacket();
        Intake_Motor = Robot.opMode.hardwareMap.get(DcMotorEx.class, "Intake_Motor");
        Intake_Motor.setDirection(DcMotor.Direction.REVERSE);
        lastPower = 0;
    }

    @Override
    public void periodic() {
        packet.put("Intake/Power", Intake_Motor.getPower());
        packet.put("Intake/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("Currents/Intake", Intake_Motor.getCurrent(CurrentUnit.AMPS));
        Robot.logPacket(packet);
    }

    private void setPower(double power) {
        Intake_Motor.setPower(power);
        lastPower = power;
    }

    public boolean isRunning() {
        return Math.abs(lastPower) > 0.02;
    }

    public Command runIntake() {
        return new IntakeDrive();
    }

    public Command runOuttake() {
        return new IntakeDriveOut();
    }

    private class IntakeDrive extends CommandBase {
        public IntakeDrive() {
            addRequirements(Robot.intake);
        }
        @Override
        public void initialize() {
            setPower(Robot.RobotConfig.INTAKE_SPEED);
        }
        @Override
        public void end(boolean interrupted) {
            setPower(0);
        }
    }

    private class IntakeDriveOut extends CommandBase {
        public IntakeDriveOut() {
            addRequirements(Robot.intake);
        }
        @Override
        public void initialize() {
            setPower(-Robot.RobotConfig.INTAKE_SPEED);
        }
        @Override
        public void end(boolean interrupted) {
            setPower(0);
        }
    }
}
