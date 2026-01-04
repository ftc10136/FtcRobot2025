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

public class Shooter extends SubsystemBase {
    private final DcMotorEx TurretShooterMotor;
    private final TelemetryPacket packet;

    public Shooter() {
        packet = new TelemetryPacket();
        TurretShooterMotor = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "TurretShooterMotor");
        TurretShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        TurretShooterMotor.setVelocityPIDFCoefficients(50, 0.05, 0, 12.2);
        TurretShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        double veloTicksPerSec = TurretShooterMotor.getVelocity();
        //there are 28 pulses per revolution
        double veloRPM = veloTicksPerSec * 60/28;
        packet.put("Shooter/VelocityTicksSec", veloTicksPerSec);
        packet.put("Shooter/VelocityRpm", veloRPM);
        packet.put("Shooter/Voltage", TurretShooterMotor.getPower());
        packet.put("Shooter/Current", TurretShooterMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("Shooter/Command", RobotUtil.getCommandName(getCurrentCommand()));
        Robot.logPacket(packet);
    }

    private void setRpmMotor(double rpm) {
        TurretShooterMotor.setVelocity((28. / 60) * rpm);
    }

    public Command setRpm(double rpm) {
        return new SetRpm(rpm);
    }

    private class SetRpm extends CommandBase {
        double rpm;
        public SetRpm(double rpm) {
            this.rpm = rpm;
            addRequirements(Robot.shooter);
        }
        @Override
        public void execute() {
            setRpmMotor(rpm);
        }
    }
}
