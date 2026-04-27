package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
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
    private final DcMotorEx TurretShooterMotor2;
    private final TelemetryPacket packet;
    private final InterpolatingDoubleTreeMap shootingTable;
    private boolean atTarget;
    private double smoothRpm;

    public Shooter() {
        packet = new TelemetryPacket();
        //batteryVoltage = Robot.opMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        TurretShooterMotor = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "TurretShooterMotor");
        TurretShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        TurretShooterMotor.setVelocityPIDFCoefficients(55, 0.04, 0, 12.2);
        TurretShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurretShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TurretShooterMotor2 = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "TurretShooterMotor2");
        TurretShooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        TurretShooterMotor2.setVelocityPIDFCoefficients(55, 0.04, 0, 12.2);
        TurretShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurretShooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		smoothRpm = 0;
        atTarget = false;

        //shot rpm table, input distance in inches, output rpm
        //values are from testing calibrateShot command at different distances
        //also set HoodAngle.shootingTable for hood angles
        shootingTable = new InterpolatingDoubleTreeMap();
        shootingTable.put(35.35, 2785.);
        shootingTable.put(47.93, 2850.);
        shootingTable.put(60.1, 3000.);
        shootingTable.put(67.59, 3100.); //48"
        shootingTable.put(79.8, 3300.); //60"
        shootingTable.put(92.09, 3550.); //72"
        shootingTable.put(104.78, 3700.); //84
        shootingTable.put(116.58, 3900.); //96
        shootingTable.put(132.33, 4050.); //108
        shootingTable.put(143.21, 4150.); //120
        shootingTable.put(151.91, 4200.); //132
        shootingTable.put(164.51, 4400.); //144
        shootingTable.put(175.52, 4500.); //156
    }

    @Override
    public void periodic() {
        double veloTicksPerSec = TurretShooterMotor.getVelocity();
        //there are 28 pulses per revolution
        double veloRPM = veloTicksPerSec * 60 / 28;
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
        packet.put("Currents/Shooter2", TurretShooterMotor2.getCurrent(CurrentUnit.AMPS));
        Robot.logPacket(packet);
    }

    private void setRpmMotor(double rpm) {
        packet.put("Shooter/TargetRpm", rpm);

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
        double deltaRpm = rpm - smoothRpm;
        if (deltaRpm > 120) {
            //need to command a bigger power to catch up faster.  (This might be better P tuning)
            double power = (rpm / 6000.) * 1.22;
            TurretShooterMotor.setPower(power);
            TurretShooterMotor2.setPower(power);
        }
        else if (deltaRpm < -120) {
            //need to slow down hard by commanding a slower motor power.  When the power is too far off,
            //the rev controller drops to 0 power, which would be coast or brake mode kicking in,
            //not motor forcing slowdown
            double power = (rpm / 6000.) * 0.93;
            TurretShooterMotor.setPower(power);
            TurretShooterMotor2.setPower(power);
        } else {
            TurretShooterMotor.setVelocity((28. / 60) * rpm);
            TurretShooterMotor2.setVelocity((28. / 60) * rpm);
        }
        atTarget = Math.abs(deltaRpm) < 50;
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
    public Command setRpmWithFinished(double rpm) {
        return new SetRpmWithFinished(rpm);
    }

    public Command preShotRpm(Pose shootingPose) {
        return new PreShotRpm(shootingPose);
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

    private class SetRpmWithFinished extends CommandBase {
        double rpm;
        public SetRpmWithFinished(double rpm) {
            this.rpm = rpm;
            addRequirements(Robot.shooter);
        }
        @Override
        public void execute() {
            setRpmMotor(rpm);
        }
        @Override
        public boolean isFinished() {
            return atTarget;
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

    private class PreShotRpm extends CommandBase {
        Pose pose;
        double rpm;
        public PreShotRpm(Pose shootingPose) {
            this.pose = shootingPose;
            addRequirements(Robot.shooter);
        }
        @Override
        public void initialize() {
            double dist = DrivetrainPP.getGoalTarget(pose).GoalDistance;
            rpm = shootingTable.get(dist);
            setRpmMotor(rpm);
        }
        @Override
        public void execute() {
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
