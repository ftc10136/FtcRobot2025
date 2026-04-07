package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.RobotUtil;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Shooter extends SubsystemBase {
    private final DcMotorEx TurretShooterMotor;
    private final DcMotorEx TurretShooterMotor2;
    private final TelemetryPacket packet;
    private final InterpolatingDoubleTreeMap shootingTable;
    private boolean atTarget;
    private double veloRPM;

    public Shooter() {
        packet = new TelemetryPacket();
        TurretShooterMotor = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "TurretShooterMotor");
        TurretShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        TurretShooterMotor.setVelocityPIDFCoefficients(50, 0.05, 0, 12.2);
        TurretShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurretShooterMotor2 = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "TurretShooterMotor2");
        TurretShooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        TurretShooterMotor2.setVelocityPIDFCoefficients(50, 0.05, 0, 12.2);
        TurretShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atTarget = false;

        //shot rpm table, input distance in inches, output rpm
        //values are from testing calibrateShot command at different distances
        //also set HoodAngle.shootingTable for hood angles
        shootingTable = new InterpolatingDoubleTreeMap();
        shootingTable.put(34.73, 2528.);
        shootingTable.put(46.68, 2742.);
        shootingTable.put(58.66, 2800.);
        shootingTable.put(70.57, 2975.);
        shootingTable.put(82.5, 3200.);
        shootingTable.put(94.68, 3525.);
        shootingTable.put(106.56, 3800.);
        shootingTable.put(118.46, 3920.);
        shootingTable.put(129.94, 4075.);
        shootingTable.put(142.32, 4400.);
        shootingTable.put(154., 4250.);
        shootingTable.put(166.28, 4300.);
    }

    @Override
    public void periodic() {
        double veloTicksPerSec = TurretShooterMotor.getVelocity();
        //there are 28 pulses per revolution
        veloRPM = veloTicksPerSec * 60/28;
        double veloTicksPerSec2 = TurretShooterMotor2.getVelocity();
        //there are 28 pulses per revolution
        double veloRPM2 = veloTicksPerSec2 * 60/28;
        packet.put("Shooter/VelocityTicksSec1", veloTicksPerSec);
        packet.put("Shooter/VelocityRpm1", veloRPM);
        packet.put("Shooter/VelocityRpm2", veloRPM2);
        //packet.put("Shooter/Voltage1", TurretShooterMotor.getPower());
        //packet.put("Shooter/Current1", TurretShooterMotor.getCurrent(CurrentUnit.AMPS));
        //packet.put("Shooter/Voltage2", TurretShooterMotor2.getPower());
        //packet.put("Shooter/Current2", TurretShooterMotor2.getCurrent(CurrentUnit.AMPS));
        packet.put("Shooter/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("Shooter/AtTarget", atTarget);
        Robot.logPacket(packet);
    }

    private void setRpmMotor(double rpm) {
        TurretShooterMotor.setVelocity((28. / 60) * rpm);
        TurretShooterMotor2.setVelocity((28. / 60) * rpm);
        atTarget = Math.abs(rpm - veloRPM) < 50;
    }

    public boolean atTarget() {
        return atTarget;
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
        @Override
        public void end(boolean interrupted) {
            setRpmMotor(0);
        }
    }

    private class AutoShotRpm extends CommandBase {
        public AutoShotRpm() {
            addRequirements(Robot.shooter);
        }
        @Override
        public void execute() {
            double dist = Robot.drivetrain.getGoalDistance();
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
