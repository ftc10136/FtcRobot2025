package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class Drivetrain {
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;
    private final double GamePad1Speed;
    private final IMU imu;

    public Drivetrain() {
        GamePad1Speed = 0.85;
        leftFront = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = (DcMotorEx)Robot.opMode.hardwareMap.get(DcMotor.class, "rightRear");
        imu = Robot.opMode.hardwareMap.get(IMU.class, "imu");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setTargetPositionTolerance(40);
        leftRear.setTargetPositionTolerance(40);
        rightRear.setTargetPositionTolerance(40);
        rightFront.setTargetPositionTolerance(40);
    }

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

        Drive2 = Range.clip(Math.sqrt(Math.pow(Robot.opMode.gamepad1.left_stick_y, 2) + Math.pow(Robot.opMode.gamepad1.left_stick_x, 2)), 0, 1);
        GamePadDegree = Math.atan2(-Math.pow(Robot.opMode.gamepad1.left_stick_y, 3), Math.pow(Robot.opMode.gamepad1.left_stick_x, 3)) / Math.PI * 180;
        Movement = GamePadDegree - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        Strafe = Math.cos(Movement / 180 * Math.PI) * Drive2;
        Forward = Math.sin(Movement / 180 * Math.PI) * Drive2;
        Turn = Robot.opMode.gamepad1.right_stick_x * -0.5;
        if (Robot.opMode.gamepad1.left_trigger >= 0.2) {
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
    }
}
