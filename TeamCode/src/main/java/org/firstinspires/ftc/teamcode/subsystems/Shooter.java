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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Shooter extends SubsystemBase {
    private final DcMotorEx TurretShooterMotor;
    private final TelemetryPacket packet;
    private final InterpolatingDoubleTreeMap shootingTable;

    public Shooter() {
        packet = new TelemetryPacket();
        TurretShooterMotor = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "TurretShooterMotor");
        TurretShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        TurretShooterMotor.setVelocityPIDFCoefficients(50, 0.05, 0, 12.2);
        TurretShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //shot rpm table, input distance in inches, output rpm
        //values are from testing calibrateShot command at different distances
        //also set HoodAngle.shootingTable for hood angles
        shootingTable = new InterpolatingDoubleTreeMap();
        shootingTable.put(16., 2700.);
        shootingTable.put(34.5, 2700.);
        shootingTable.put(50., 2800.);
        shootingTable.put(62., 3000.);
        shootingTable.put(82., 3300.);
        shootingTable.put(97., 3600.);
        shootingTable.put(113., 3900.);
        //TODO, need to get table to 144", to get the back corner shots
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

    public Command calibrateShot() {
        return new CalibrateShot();
    }

    public Command autoShotRpm() {
        return new AutoShotRpm();
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

    private class AutoShotRpm extends CommandBase {
        public AutoShotRpm() {
            addRequirements(Robot.shooter);
        }
        @Override
        public void execute() {
            double dist = Robot.vision.getDistance();
            var rpm = shootingTable.get(dist);
            setRpmMotor(rpm);
        }
    }

    private class CalibrateShot extends CommandBase {
        public CalibrateShot() {
            addRequirements(Robot.shooter);
        }
        @Override
        public void execute() {
            setRpmMotor(Robot.RobotConfig.CALIBRATE_SHOT_RPM);
        }
        @Override
        public void end(boolean interrupted) {
            setRpmMotor(0);
        }
    }
}
