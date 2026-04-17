package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
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
    private double smoothRpm;

    public Shooter() {
        packet = new TelemetryPacket();
        //batteryVoltage = Robot.opMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        TurretShooterMotor = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "TurretShooterMotor");
        TurretShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        TurretShooterMotor.setVelocityPIDFCoefficients(55, 0.01, 0, 12.2);
        TurretShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurretShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretShooterMotor2 = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "TurretShooterMotor2");
        TurretShooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        TurretShooterMotor2.setVelocityPIDFCoefficients(55, 0.01, 0, 12.2);
        TurretShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurretShooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smoothRpm = 0;
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
        shootingTable.put(142.32, 4300.);
        shootingTable.put(154., 4350.);
        shootingTable.put(166.28, 4500.);
    }

    @Override
    public void periodic() {
        double veloTicksPerSec = TurretShooterMotor.getVelocity();
        //there are 28 pulses per revolution
        veloRPM = veloTicksPerSec * 60/28;
        double veloTicksPerSec2 = TurretShooterMotor2.getVelocity();
        //there are 28 pulses per revolution
        double veloRPM2 = veloTicksPerSec2 * 60/28;
        smoothRpm = (veloRPM * Robot.RobotConfig.SHOOTER_RPM_SMOOTHER) + (smoothRpm * (1-Robot.RobotConfig.SHOOTER_RPM_SMOOTHER));
        packet.put("Shooter/VelocityTicksSec1", veloTicksPerSec);
        packet.put("Shooter/VelocityRpm1", veloRPM);
        packet.put("Shooter/VelocityRpm2", veloRPM2);
        packet.put("Shooter/SmoothRpm", smoothRpm);
        //packet.put("Shooter/Voltage1", TurretShooterMotor.getPower());
        //packet.put("Shooter/Current1", TurretShooterMotor.getCurrent(CurrentUnit.AMPS));
        //packet.put("Shooter/Voltage2", TurretShooterMotor2.getPower());
        //packet.put("Shooter/Current2", TurretShooterMotor2.getCurrent(CurrentUnit.AMPS));
        packet.put("Shooter/Command", RobotUtil.getCommandName(getCurrentCommand()));
        packet.put("Shooter/AtTarget", atTarget);
        packet.put("Currents/Shooter1", TurretShooterMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("Currents/Shooter2", TurretShooterMotor.getCurrent(CurrentUnit.AMPS));
        Robot.logPacket(packet);
    }

    private void setRpmMotor(double rpm) {
        //software based PID
        //double feedForward = Robot.RobotConfig.SHOOTER_MOTOR_KS + (rpm * Robot.RobotConfig.SHOOTER_MOTOR_KV);
        //double feedBack = (rpm - veloRPM) * Robot.RobotConfig.SHOOTER_MOTOR_KP;
        //double power = feedForward + feedBack;
        //if (rpm > 300) {
        //    power = Range.clip(power, 0.08, 1.);
        //}
        //TurretShooterMotor.setPower(power);
        //TurretShooterMotor2.setPower(power);
        //hardware based PID shots
        TurretShooterMotor.setVelocity((28. / 60) * rpm);
        TurretShooterMotor2.setVelocity((28. / 60) * rpm);
        atTarget = Math.abs(rpm - smoothRpm) < 50;
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
