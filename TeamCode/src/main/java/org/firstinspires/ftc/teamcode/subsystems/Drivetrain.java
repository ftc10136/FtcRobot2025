package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.RobotUtil;

import java.util.HashMap;
import java.util.Map;

public class Drivetrain extends SubsystemBase {
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;
    private final double GamePad1Speed;
    private final IMU imu;
    private final TelemetryPacket packet;
    private final HashMap<String, DcMotorEx> motorList;

    public Drivetrain() {
        super();
        GamePad1Speed = 0.85;
        leftFront = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "rightRear");
        imu = Robot.opMode.hardwareMap.get(IMU.class, "imu");

        //setup logging
        packet = new TelemetryPacket();
        motorList = new HashMap<>();
        motorList.put("leftFront", leftFront);
        motorList.put("leftRear", leftRear);
        motorList.put("rightFront", rightFront);
        motorList.put("rightRear", rightRear);

        //configure motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        //common config
        for (DcMotorEx motor : motorList.values()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setTargetPositionTolerance(40);
        }
    }

    @Override
    public void periodic() {
        //In periodic, we can read data and log it, but DO NOT COMMAND MOTORS
        for (Map.Entry<String, DcMotorEx> entry : motorList.entrySet()) {
            String keyBase = "Drivetrain/" + entry.getKey();
            DcMotorEx motor = entry.getValue();
            packet.put(keyBase + "/Current", motor.getCurrent(CurrentUnit.AMPS));
            packet.put(keyBase + "/Velocity", motor.getVelocity());
            packet.put(keyBase + "/Position", motor.getCurrentPosition());
            packet.put(keyBase + "/Power", motor.getPower());
        }
        packet.put("IMU/Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        packet.put("Drivetrain/Command", RobotUtil.getCommandName(getCurrentCommand()));

        Robot.logPacket(packet);
    }

    @SuppressWarnings("unused")
    public void RobotCenteredDriving() {
        double vertical;
        double horizontal;
        double pivot;

        vertical = -Double.parseDouble(JavaUtil.formatNumber(Robot.opMode.gamepad1.left_stick_y, 2)) * GamePad1Speed;
        horizontal = -Double.parseDouble(JavaUtil.formatNumber(Robot.opMode.gamepad1.left_stick_x, 2)) * GamePad1Speed;
        pivot = Double.parseDouble(JavaUtil.formatNumber(Robot.opMode.gamepad1.right_stick_x, 2)) * 0.85 * GamePad1Speed;
        rightFront.setVelocity(((-pivot + vertical + horizontal) / 1.1) * (0.6 * Math.pow(-pivot + vertical + horizontal, 2) + 0.5) * 0 * 1500);
        rightRear.setVelocity(((-pivot + (vertical - horizontal)) / 1.1) * (0.6 * Math.pow(-pivot + (vertical - horizontal), 2) + 0.5) * 0 * 1500);
        leftFront.setVelocity(((pivot + (vertical - horizontal)) / 1.1) * (0.6 * Math.pow(pivot + (vertical - horizontal), 2) + 0.5) * 0 * 1500);
        leftRear.setVelocity(((pivot + vertical + horizontal) / 1.1) * (0.6 * Math.pow(pivot + vertical + horizontal, 2) + 0.5) * 0 * 1500);
    }

    public void FieldCentricDriving() {
        double Drive2;
        double GamePadDegree;
        double Movement;
        double Strafe;
        double Forward;
        double Turn;

        Drive2 = Range.clip(Math.sqrt(Math.pow(Robot.controls.getDriveForward(), 2) + Math.pow(Robot.controls.getDriveRight(), 2)), 0, 1);
        GamePadDegree = Math.atan2(-Math.pow(Robot.controls.getDriveForward(), 3), Math.pow(Robot.controls.getDriveRight(), 3)) / Math.PI * 180;
        Movement = GamePadDegree - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        Strafe = Math.cos(Movement / 180 * Math.PI) * Drive2;
        Forward = Math.sin(Movement / 180 * Math.PI) * Drive2;
        Turn = Robot.controls.getDriveTurn() * -0.5;
        if (Robot.controls.slowModeActive()) {
            leftFront.setVelocity(((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe)) - Turn) * 0.15 * 2800);
            rightFront.setVelocity(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) + Turn) * 0.15 * 2800);
            rightRear.setVelocity((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe) + Turn) * 0.15 * 2800);
            leftRear.setVelocity(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) - Turn) * 0.15 * 2800);
        } else {
            leftFront.setVelocity(((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe)) - Turn) * GamePad1Speed * 2800);
            rightFront.setVelocity(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) + Turn) * GamePad1Speed * 2800);
            rightRear.setVelocity((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe) + Turn) * GamePad1Speed * 2800);
            leftRear.setVelocity(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) - Turn) * GamePad1Speed * 2800);
        }

        packet.put("Drivetrain/Drive2", Drive2);
        packet.put("Drivetrain/GamePadDegree", GamePadDegree);
        packet.put("Drivetrain/Movement", Movement);
        packet.put("Drivetrain/Strafe", Strafe);
        packet.put("Drivetrain/Forward", Forward);
        packet.put("Drivetrain/Turn", Turn);
    }

    public static class TeleopDrive extends CommandBase {
        public TeleopDrive() {
            addRequirements(Robot.drivetrain);
        }
        @Override
        public void execute() {
            //Robot.drivetrain.FieldCentricDriving();
        }
    }
}
