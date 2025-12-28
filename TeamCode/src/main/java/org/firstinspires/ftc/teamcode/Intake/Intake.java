package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotUtil;

public class Intake extends SubsystemBase {
    private final DcMotor Intake_Motor;
    private final TelemetryPacket packet;

    public Intake() {
        packet = new TelemetryPacket();
        Intake_Motor = Robot.opMode.hardwareMap.get(DcMotor.class, "Intake_Motor");
        Intake_Motor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        packet.put("Intake/Power", Intake_Motor.getPower());
        packet.put("Intake/Command", RobotUtil.getCommandName(getCurrentCommand()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public Command runIntake() {
        return new IntakeDrive();
    }

    private class IntakeDrive extends CommandBase {
        public IntakeDrive() {
            addRequirements(Robot.intake);
        }
        @Override
        public void execute() {
            Intake_Motor.setPower(1);
        }
        @Override
        public void end(boolean interrupted) {
            Intake_Motor.setPower(0);
        }
    }
}
