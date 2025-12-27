package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "_10136_CompBot_TeleOp_RedStates_V04 (Blocks to Java)")
public class _10136_CompBot_TeleOp_RedStates_V04 extends LinearOpMode {

    private ColorSensor Bay1AColor_REV_ColorRangeSensor;
    private ColorSensor Bay1AColor;
    private Servo RGBBay1;
    private ColorSensor Bay2AColor_REV_ColorRangeSensor;
    private ColorSensor Bay2AColor;
    private Servo RGBBay2;
    private ColorSensor Bay3AColor_REV_ColorRangeSensor;
    private ColorSensor Bay3AColor;
    private Servo RGBBay3;
    private IMU imu;
    private Limelight3A limelight;
    private Servo RGBAlliance;
    private Servo HoodAngle;
    private Servo TurretSpin;
    private Servo Ballevator;
    private Servo Spindexer;
    private DcMotor TurretShooterMotor;
    private DcMotor Intake_Motor;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private AnalogInput BallevatorEncoder;
    private AnalogInput SpindexerEncoder;
    private Servo RGB_VisionAcquired;

    ElapsedTime ServoTimer;
    LLStatus status;
    int SpindexerColorError;
    double ShooterRPMOffset;
    ElapsedTime WheelsTimers;
    LLResult result;
    double ShooterRPM;
    int FieldCentricActive;
    double GamePad1Speed;
    int Bay1Status;
    double Bay1Shooting;
    int Bay2Status;
    double Bay2Shooting;
    int Bay3Status;
    double Bay3Shooting;
    int FloorIntakeStatus;
    int HumanIntake2;
    double OldTurretPosition;
    int TurretMode;
    double Ballevator2;
    int VisionAcquired;
    double Bay1IntakeLoading;
    double Bay2IntakeLoading;
    double Bay3IntakeLoading;
    int AbortShooting;
    int TouretManualOffset;
    int BallevatorTime;
    double YDriveFactor;
    int MotorFactor;
    double Bay1HumanLoading;
    double Bay2HumanLoading;
    double Bay3HumanLoading;
    int SpindexerTransitDelayTime;
    ElapsedTime AbortTimer;
    double XDriveFactor;
    double RotDriveFactor;
    double Accel;
    double max_speed;
    int Decel;
    List<LLResultTypes.FiducialResult> fiducialResults;
    LLResultTypes.FiducialResult fiducialResult;

    /**
     * Describe this function...
     */
    private void Bay1Sensing() {
        // RGBServo_0=Off,.0.277=Red, 0.333=Orange, 0.388=Yellow, 0.444=Sage, 0.5=Green, 0.555=Azure, 0.611=Blue, 0.666=Indigo, 0.722=Violet, 1.0=White
        // Status:0=Empty,1=Someting,2=Green,3=Purple
        if (((DistanceSensor) Bay1AColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 2) {
            if (Bay1AColor.green() > Bay1AColor.red() && Bay1AColor.green() > Bay1AColor.blue()) {
                Bay1Status = 2;
                RGBBay1.setPosition(0.5);
            } else if (Bay1AColor.blue() > Bay1AColor.red() && Bay1AColor.blue() > Bay1AColor.green()) {
                Bay1Status = 3;
                RGBBay1.setPosition(0.722);
            } else if (((DistanceSensor) Bay1AColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 2) {
                Bay1Status = 1;
                RGBBay1.setPosition(0.333);
            } else {
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Bay2Sensing() {
        // Status:0=Empty,1=Someting,2=Green,3=Purple
        if (((DistanceSensor) Bay2AColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 2) {
            if (Bay2AColor.green() > Bay2AColor.red() && Bay2AColor.green() > Bay2AColor.blue()) {
                Bay2Status = 2;
                RGBBay2.setPosition(0.5);
            } else if (Bay2AColor.blue() > Bay2AColor.red() && Bay2AColor.blue() > Bay2AColor.green()) {
                Bay2Status = 3;
                RGBBay2.setPosition(0.722);
            } else if (((DistanceSensor) Bay2AColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 2) {
                Bay2Status = 1;
                RGBBay2.setPosition(0.333);
            } else {
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Driving() {
        if (FieldCentricActive == 1) {
            FieldCentricDriving();
        } else if (FieldCentricActive == 0) {
            RobotCenteredDriving();
        }
    }

    /**
     * Describe this function...
     */
    private void Bay3Sensing() {
        // Status:0=Empty,1=Someting,2=Green,3=Purple
        if (((DistanceSensor) Bay3AColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 2) {
            if (Bay3AColor.green() > Bay3AColor.red() && Bay3AColor.green() > Bay3AColor.blue()) {
                Bay3Status = 2;
                RGBBay3.setPosition(0.5);
            } else if (Bay3AColor.blue() > Bay3AColor.red() && Bay3AColor.blue() > Bay3AColor.green()) {
                Bay3Status = 3;
                RGBBay3.setPosition(0.722);
            } else if (((DistanceSensor) Bay3AColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 2) {
                Bay3Status = 1;
                RGBBay3.setPosition(0.333);
            } else {
            }
        }
    }

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        int AllianceColor;
        double SpindexerPosition;
        int ShooterOn;
        int AutonMode;

        Bay1AColor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "Bay1A-Color");
        Bay1AColor = hardwareMap.get(ColorSensor.class, "Bay1A-Color");
        RGBBay1 = hardwareMap.get(Servo.class, "RGB-Bay1");
        Bay2AColor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "Bay2A-Color");
        Bay2AColor = hardwareMap.get(ColorSensor.class, "Bay2A-Color");
        RGBBay2 = hardwareMap.get(Servo.class, "RGB-Bay2");
        Bay3AColor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "Bay3A-Color");
        Bay3AColor = hardwareMap.get(ColorSensor.class, "Bay3A-Color");
        RGBBay3 = hardwareMap.get(Servo.class, "RGB-Bay3");
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        RGBAlliance = hardwareMap.get(Servo.class, "RGB-Alliance");
        HoodAngle = hardwareMap.get(Servo.class, "HoodAngle");
        TurretSpin = hardwareMap.get(Servo.class, "TurretSpin");
        Ballevator = hardwareMap.get(Servo.class, "Ballevator");
        Spindexer = hardwareMap.get(Servo.class, "Spindexer");
        TurretShooterMotor = hardwareMap.get(DcMotor.class, "TurretShooterMotor");
        Intake_Motor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        BallevatorEncoder = hardwareMap.get(AnalogInput.class, "BallevatorEncoder");
        SpindexerEncoder = hardwareMap.get(AnalogInput.class, "SpindexerEncoder");
        RGB_VisionAcquired = hardwareMap.get(Servo.class, "RGB_VisionAcquired");

        // Put initialization blocks here.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        telemetry.setMsTransmissionInterval(11);
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        AllianceColor = 0;
        RGBAlliance.setPosition(0.29);
        limelight.pipelineSwitch(2);
        Ballevator2 = 0.246;
        BallevatorTime = 500;
        FieldCentricActive = 1;
        GamePad1Speed = 0.85;
        SpindexerPosition = 0.4072;
        Bay1Status = 0;
        Bay2Status = 0;
        Bay3Status = 0;
        ShooterOn = 0;
        SpindexerColorError = 0;
        SpindexerTransitDelayTime = 500;
        TouretManualOffset = 0;
        ShooterRPMOffset = 0;
        TurretMode = 1;
        AbortShooting = 0;
        MotorServoInits();
        HoodAngle.scaleRange(0.2, 0.8);
        HoodAngle.setPosition(0.35);
        TurretSpin.setPosition(0.5);
        Ballevator.setPosition(0);
        Spindexer.setPosition(SpindexerPosition);
        // RGBServo_0=Off,.0.277=Red, 0.333=Orange, 0.388=Yellow, 0.444=Sage, 0.5=Green, 0.555=Azure, 0.611=Blue, 0.666=Indigo, 0.722=Violet, 1.0=White
        // LimelightPipelines: 1=20/BlueAlliance, 2=24/RedAlliance, 3=20,21,22
        while (!opModeIsActive()) {
            status = limelight.getStatus();
            result = limelight.getLatestResult();
            if (result != null) {
                // Access general information.
                // Access fiducial results.
                fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducialResult_item : fiducialResults) {
                    fiducialResult = fiducialResult_item;
                    telemetry.addData("Fiducial", "ID: " + fiducialResult.getFiducialId() + ", Family: " + fiducialResult.getFamily() + ", X: " + JavaUtil.formatNumber(fiducialResult.getTargetXDegrees(), 2) + ", Y: " + JavaUtil.formatNumber(fiducialResult.getTargetYDegrees(), 2));
                    AutonMode = fiducialResult.getFiducialId();
                    telemetry.update();
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            Bay1Sensing();
            Bay2Sensing();
            Bay3Sensing();
        }
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            ShooterRPM = 3000;
            Bay1Shooting = SpindexerPosition;
            Bay2Shooting = Bay1Shooting + 0.1851;
            Bay3Shooting = Bay1Shooting + 0.372;
            Bay1IntakeLoading = Bay1Shooting + 0.095;
            Bay2IntakeLoading = Bay1Shooting + 0.2801;
            Bay3IntakeLoading = Bay1Shooting + 0.4633;
            Bay1HumanLoading = Bay1Shooting + 0.4255;
            Bay2HumanLoading = Bay1Shooting + 0.0483;
            Bay3HumanLoading = Bay1Shooting + 0.24;
            while (opModeIsActive()) {
                ((DcMotorEx) TurretShooterMotor).setVelocity((28 / 60) * ShooterRPM);
                // Put loop blocks here.
                Bay1Sensing();
                Bay2Sensing();
                Bay3Sensing();
                BaysFullyLoaded();
                HumanIntake();
                FloorIntakeControl();
                GamePad1();
                GamePad2();
                LimeLight();
                AutoTrackAprilTag();
                Driving();
                telemetry.addData("VisionAcquired", VisionAcquired);
                telemetry.addData("RPMDesired", ShooterRPM);
                telemetry.addData("TurretSpin", TurretSpin.getPosition());
                telemetry.addData("Spindexer", Spindexer.getPosition());
                telemetry.addData("Bay1Status", Bay1Status);
                telemetry.addData("Bay2Status", Bay2Status);
                telemetry.addData("Bay3Status", Bay3Status);
                telemetry.addData("HoodPosition", HoodAngle.getPosition());
                telemetry.update();
                // Bay Status:0=Empty,1=Someting,2=Green,3=Purple
            }
        }
    }

    /**
     * Describe this function...
     */
    private void MotorServoInits() {
        int DriveX;
        int DriveY;
        int DriveRot;

        Spindexer.setDirection(Servo.Direction.REVERSE);
        TurretSpin.setDirection(Servo.Direction.FORWARD);
        Intake_Motor.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        Ballevator.setDirection(Servo.Direction.FORWARD);
        ((DcMotorEx) leftFront).setTargetPositionTolerance(40);
        ((DcMotorEx) leftRear).setTargetPositionTolerance(40);
        ((DcMotorEx) rightRear).setTargetPositionTolerance(40);
        ((DcMotorEx) rightFront).setTargetPositionTolerance(40);
        MotorFactor = 1;
        XDriveFactor = 44.888;
        YDriveFactor = 42.188;
        RotDriveFactor = 9.708;
        max_speed = 0.7;
        DriveX = 0;
        DriveY = 0;
        DriveRot = 0;
        Accel = 0.04;
        Decel = 2600;
        AbortTimer = new ElapsedTime();
        ServoTimer = new ElapsedTime();
        WheelsTimers = new ElapsedTime();
        ((DcMotorEx) TurretShooterMotor).setVelocityPIDFCoefficients(50, 0.05, 0, 12.2);
        TurretShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void RobotCenteredDriving() {
        double vertical;
        double horizontal;
        double pivot;

        vertical = -Double.parseDouble(JavaUtil.formatNumber(gamepad1.left_stick_y, 2)) * GamePad1Speed;
        horizontal = -Double.parseDouble(JavaUtil.formatNumber(gamepad1.left_stick_x, 2)) * GamePad1Speed;
        pivot = Double.parseDouble(JavaUtil.formatNumber(gamepad1.right_stick_x, 2)) * 0.85 * GamePad1Speed;
        ((DcMotorEx) rightFront).setVelocity(((-pivot + vertical + horizontal) / 1.1) * (0.6 * Math.pow(-pivot + vertical + horizontal, 2) + 0.5) * 0 * 1500);
        ((DcMotorEx) rightRear).setVelocity(((-pivot + (vertical - horizontal)) / 1.1) * (0.6 * Math.pow(-pivot + (vertical - horizontal), 2) + 0.5) * 0 * 1500);
        ((DcMotorEx) leftFront).setVelocity(((pivot + (vertical - horizontal)) / 1.1) * (0.6 * Math.pow(pivot + (vertical - horizontal), 2) + 0.5) * 0 * 1500);
        ((DcMotorEx) leftRear).setVelocity(((pivot + vertical + horizontal) / 1.1) * (0.6 * Math.pow(pivot + vertical + horizontal, 2) + 0.5) * 0 * 1500);
    }

    /**
     * Describe this function...
     */
    private void GamePad2() {
        int SpindexerManualOffset;

        if (gamepad2.dpad_up_was_pressed()) {
            ShooterRPMOffset = Math.min(Math.max(ShooterRPMOffset + 50, -200), 200);
        } else if (gamepad2.dpad_down_was_pressed()) {
            ShooterRPMOffset = Math.min(Math.max(ShooterRPMOffset - 50, -200), 200);
        }
        if (gamepad2.left_trigger > 0.2) {
            SpindexerManualOffset = 1;
            Spindexer.setPosition(Spindexer.getPosition() + SpindexerManualOffset);
            ServoTimePause(250);
            Spindexer.setPosition(Spindexer.getPosition() - SpindexerManualOffset);
        } else if (gamepad2.right_trigger > 0.2) {
            SpindexerManualOffset = -1;
            Spindexer.setPosition(Spindexer.getPosition() + SpindexerManualOffset);
            ServoTimePause(250);
            Spindexer.setPosition(Spindexer.getPosition() - SpindexerManualOffset);
        } else {
            SpindexerManualOffset = 0;
        }
        if (gamepad2.x_was_pressed()) {
            // HumanIntake
            HumanIntake2 = 1;
            FloorIntakeStatus = 0;
            HumanIntake();
        }
        if (gamepad2.y_was_pressed()) {
            // ShootingAbort
            AbortShooting = 1;
            AbortTimer.reset();
        }
        if (AbortTimer.milliseconds() > 2000) {
            AbortShooting = 0;
        }
        if (gamepad2.a) {
            FloorIntakeStatus = 1;
            TurretMode = 1;
            HumanIntake2 = 0;
            // Resets BayStatus at start of intake
            Bay1Status = 0;
            Bay2Status = 0;
            Bay3Status = 0;
            Intake_Motor.setPower(1);
        }
    }

    /**
     * Describe this function...
     */
    private void GamePad1() {
        if (gamepad1.guide) {
            imu.resetYaw();
        }
        if (gamepad1.back && FieldCentricActive == 1) {
            FieldCentricActive = 0;
        } else if (gamepad1.back && FieldCentricActive == 0) {
            FieldCentricActive = 1;
        }
        if (gamepad1.y_was_pressed()) {
            ShootAll();
        }
        if (gamepad1.x_was_pressed()) {
            // HiumanIntake
            HumanIntake2 = 1;
            FloorIntakeStatus = 0;
            HumanIntake();
        }
        if (gamepad1.dpad_up_was_pressed()) {
            ShooterRPMOffset = Math.min(Math.max(ShooterRPMOffset + 50, -200), 200);
        } else if (gamepad1.dpad_down_was_pressed()) {
            ShooterRPMOffset = Math.min(Math.max(ShooterRPMOffset - 50, -200), 200);
        }
        if (gamepad1.left_bumper_was_pressed()) {
            ShootGreen();
        }
        if (gamepad1.right_bumper_was_pressed()) {
            ShootPurple();
        }
        if (gamepad1.a) {
            FloorIntakeStatus = 1;
            TurretMode = 1;
            HumanIntake2 = 0;
            // Resets BayStatus at start of intake
            Bay1Status = 0;
            Bay2Status = 0;
            Bay3Status = 0;
            Intake_Motor.setPower(1);
        }
    }

    /**
     * Describe this function...
     */
    private void ServoTimePause(int ServoTimePause2) {
        ServoTimer.reset();
        while (!(ServoTimer.milliseconds() > ServoTimePause2)) {
            Driving();
        }
    }

    /**
     * Describe this function...
     */
    private void FieldCentricDriving() {
        double Drive2;
        double GamePadDegree;
        double Movement;
        double Strafe;
        double Forward;
        double Turn;

        Drive2 = Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)), 0, 1);
        GamePadDegree = Math.atan2(-Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.left_stick_x, 3)) / Math.PI * 180;
        Movement = GamePadDegree - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        Strafe = Math.cos(Movement / 180 * Math.PI) * Drive2;
        Forward = Math.sin(Movement / 180 * Math.PI) * Drive2;
        Turn = gamepad1.right_stick_x * -0.75;
        if (gamepad1.left_trigger >= 0.2) {
            ((DcMotorEx) leftFront).setVelocity(((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe)) - Turn) * 0.2 * 2800);
            ((DcMotorEx) rightFront).setVelocity(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) + Turn) * 0.2 * 2800);
            ((DcMotorEx) rightRear).setVelocity((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe) + Turn) * 0.2 * 2800);
            ((DcMotorEx) leftRear).setVelocity(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) - Turn) * 0.2 * 2800);
        } else {
            ((DcMotorEx) leftFront).setVelocity(((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe)) - Turn) * GamePad1Speed * 2800);
            ((DcMotorEx) rightFront).setVelocity(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) + Turn) * GamePad1Speed * 2800);
            ((DcMotorEx) rightRear).setVelocity((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe) + Turn) * GamePad1Speed * 2800);
            ((DcMotorEx) leftRear).setVelocity(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) - Turn) * GamePad1Speed * 2800);
        }
    }

    /**
     * Describe this function...
     */
    private void StopAndReset() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void Drive(double DriveX, double DriveY, double DriveRot) {
        // Forward=45 (Power All Same Direction)
        // Sideway=46.5 (Diagonals Flipped)
        // Rotate90=1060 (One Side Flipped)
        StopAndReset();
        leftFront.setTargetPosition((int) (DriveY * YDriveFactor * MotorFactor + DriveX * XDriveFactor * MotorFactor + DriveRot * -RotDriveFactor * MotorFactor));
        leftRear.setTargetPosition((int) (DriveY * YDriveFactor * MotorFactor + DriveX * -XDriveFactor * MotorFactor + DriveRot * -RotDriveFactor * MotorFactor));
        rightFront.setTargetPosition((int) (DriveY * YDriveFactor * MotorFactor + DriveX * -XDriveFactor * MotorFactor + DriveRot * RotDriveFactor * MotorFactor));
        rightRear.setTargetPosition((int) (DriveY * YDriveFactor * MotorFactor + DriveX * XDriveFactor * MotorFactor + DriveRot * RotDriveFactor * MotorFactor));
        RunToPositionGo();
        StopAndReset();
    }

    /**
     * Describe this function...
     */
    private void FloorIntakeControl() {
        // IntakeStatus: 0=Off, 1=On
        if (FloorIntakeStatus == 1) {
            if (Bay1Status == 0 && Bay2Status == 0 && Bay3Status == 0) {
                // All Empty
                Spindexer.setPosition(Bay1IntakeLoading);
            } else if (Bay1Status == 0 && Bay2Status == 0 && Bay3Status >= 1) {
                // Bay1&2 Emply
                Spindexer.setPosition(Bay1IntakeLoading);
            } else if (Bay1Status == 0 && Bay2Status >= 1 && Bay3Status == 0) {
                // Bay1&3 Empty
                Spindexer.setPosition(Bay1IntakeLoading);
            } else if (Bay1Status == 0 && Bay2Status >= 1 && Bay3Status >= 1) {
                // Bay1 Empty
                Spindexer.setPosition(Bay1IntakeLoading);
            } else if (Bay1Status >= 1 && Bay2Status == 0 && Bay3Status == 0) {
                // Bay2&3 Empty
                Spindexer.setPosition(Bay2IntakeLoading);
            } else if (Bay1Status >= 1 && Bay2Status == 0 && Bay3Status >= 1) {
                // Bay2 Empty
                Spindexer.setPosition(Bay2IntakeLoading);
            } else if (Bay1Status >= 1 && Bay2Status >= 1 && Bay3Status == 0) {
                // Bay3 Empty
                Spindexer.setPosition(Bay3IntakeLoading);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void HumanIntake() {
        // IntakeStatus: 0=Off, 1=On
        if (HumanIntake2 == 1) {
            if (TurretMode == 1) {
                OldTurretPosition = TurretSpin.getPosition();
            }
            TurretMode = 0;
            TurretSpin.setPosition(1);
            if (Bay1Status == 0 && Bay2Status == 0 && Bay3Status == 0) {
                // All Empty
                Spindexer.setPosition(Bay1HumanLoading);
            } else if (Bay1Status == 0 && Bay2Status == 0 && Bay3Status >= 1) {
                // Bay1&2 Emply
                Spindexer.setPosition(Bay1HumanLoading);
            } else if (Bay1Status == 0 && Bay2Status >= 1 && Bay3Status == 0) {
                // Bay1&3 Empty
                Spindexer.setPosition(Bay1HumanLoading);
            } else if (Bay1Status == 0 && Bay2Status >= 1 && Bay3Status >= 1) {
                // Bay1 Empty
                Spindexer.setPosition(Bay1HumanLoading);
            } else if (Bay1Status >= 1 && Bay2Status == 0 && Bay3Status == 0) {
                // Bay2&3 Empty
                Spindexer.setPosition(Bay2HumanLoading);
            } else if (Bay1Status >= 1 && Bay2Status == 0 && Bay3Status >= 1) {
                // Bay2 Empty
                Spindexer.setPosition(Bay2HumanLoading);
            } else if (Bay1Status >= 1 && Bay2Status >= 1 && Bay3Status == 0) {
                // Bay3 Empty
                Spindexer.setPosition(Bay3HumanLoading);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void RunToPositionGo() {
        WheelsTimers.reset();
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(0.15);
        rightFront.setPower(0.15);
        rightRear.setPower(0.15);
        leftRear.setPower(0.15);
        while (!(!leftFront.isBusy() && !rightRear.isBusy() && !rightFront.isBusy() && !rightRear.isBusy())) {
            if (WheelsTimers.milliseconds() > 5000) {
                break;
            }
            leftFront.setPower(Math.min(Math.max(leftFront.getPower() + Accel, 0.1), Math.min(Math.max(Math.pow(Math.abs(leftFront.getTargetPosition() - leftFront.getCurrentPosition()) / Decel, 0.5), 0.1), max_speed)));
            rightFront.setPower(Math.min(Math.max(rightFront.getPower() + Accel, 0.1), Math.min(Math.max(Math.pow(Math.abs(rightFront.getTargetPosition() - rightFront.getCurrentPosition()) / Decel, 0.5), 0.1), max_speed)));
            rightRear.setPower(Math.min(Math.max(rightRear.getPower() + Accel, 0.1), Math.min(Math.max(Math.pow(Math.abs(rightRear.getTargetPosition() - rightRear.getCurrentPosition()) / Decel, 0.5), 0.1), max_speed)));
            leftRear.setPower(Math.min(Math.max(leftRear.getPower() + Accel, 0.1), Math.min(Math.max(Math.pow(Math.abs(leftRear.getTargetPosition() - leftRear.getCurrentPosition()) / Decel, 0.5), 0.1), max_speed)));
            telemetry.addData("LeftFrontDelta", leftFront.getCurrentPosition() - leftFront.getTargetPosition());
            telemetry.addData("RightFrontDelta", rightFront.getCurrentPosition() - rightFront.getTargetPosition());
            telemetry.addData("LeftRearDelta", leftRear.getCurrentPosition() - leftRear.getTargetPosition());
            telemetry.addData("RightRearDelta", rightRear.getCurrentPosition() - rightRear.getTargetPosition());
            telemetry.update();
            sleep(10);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void LimeLight() {
        Pose3D botpose;
        double captureLatency;
        double targetingLatency;
        double X_Error;
        double GainFactor;
        double CorrectionNeeded;

        status = limelight.getStatus();
        result = limelight.getLatestResult();
        if (result != null) {
            // Access general information.
            VisionAcquired = 1;
            botpose = result.getBotpose();
            captureLatency = result.getCaptureLatency();
            targetingLatency = result.getTargetingLatency();
            X_Error = result.getTx();
            if (Math.abs(X_Error) < 0.8) {
                GainFactor = 0;
            } else if (Math.abs(X_Error) < 10) {
                GainFactor = 0.8;
            } else {
                GainFactor = 1;
            }
            CorrectionNeeded = X_Error * -0.005 * GainFactor;
            if (TurretMode == 1) {
                TurretSpin.setPosition(TurretSpin.getPosition() - CorrectionNeeded);
            }
            // Access fiducial results.
            fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult_item2 : fiducialResults) {
                fiducialResult = fiducialResult_item2;
                telemetry.addData("Fiducial", "ID: " + fiducialResult.getFiducialId() + ", Family: " + fiducialResult.getFamily() + ", X: " + JavaUtil.formatNumber(fiducialResult.getTargetXDegrees(), 2) + ", Y: " + JavaUtil.formatNumber(fiducialResult.getTargetYDegrees(), 2));
            }
        } else {
        }
    }

    /**
     * Describe this function...
     */
    private void BaysFullyLoaded() {
        int BaysAllLoaded;

        if (Bay1Status >= 1 && Bay2Status >= 1 && Bay3Status >= 1) {
            if (HumanIntake2 == 1) {
                TurretSpin.setPosition(OldTurretPosition);
            }
            BaysAllLoaded = 1;
            TurretMode = 1;
            HumanIntake2 = 0;
            Intake_Motor.setPower(0);
            Spindexer.setPosition(Bay3Shooting);
        } else {
            Intake_Motor.setPower(1);
        }
    }

    /**
     * Describe this function...
     */
    private void Shoot() {
        WheelsTimers.reset();
        if (result.getBotposeAvgDist() > 0.5) {
            while (!(Math.abs((((DcMotorEx) TurretShooterMotor).getVelocity() * 60) / 28 - ShooterRPM) < 200)) {
                if (WheelsTimers.milliseconds() > 1500) {
                    break;
                }
                telemetry.addData("ShooterRPM Error", (((DcMotorEx) TurretShooterMotor).getVelocity() * 60) / 28 - ShooterRPM);
                telemetry.addData("Actual RPM", (((DcMotorEx) TurretShooterMotor).getVelocity() * 60) / 28);
                telemetry.addData("Target RPM", ShooterRPM);
                telemetry.update();
            }
            Shoot2();
        } else {
            while (!(WheelsTimers.milliseconds() > 1500)) {
            }
            Shoot2();
        }
    }

    /**
     * Describe this function...
     */
    private void Shoot2() {
        ServoTimer.reset();
        Ballevator.setPosition(Ballevator2 + 0.005);
        while (!(BallevatorEncoder.getVoltage() < 2.297)) {
            Driving();
            if (ServoTimer.milliseconds() > BallevatorTime) {
                break;
            }
        }
        ServoTimer.reset();
        Ballevator.setPosition(0);
        while (!(BallevatorEncoder.getVoltage() > 2.9)) {
            Driving();
            if (ServoTimer.milliseconds() > BallevatorTime) {
                break;
            }
        }
    }

    /**
     * Describe this function...
     */
    private void ShootAll() {
        SpindexerColorError = 0;
        if (Bay1Status > 0.5 && Bay2Status > 0.5 && Bay3Status > 0.5) {
            GamePad2();
            if (AbortShooting == 0) {
                SpindexerPositon(Bay3Shooting);
                Shoot();
                Bay3Status = 0;
                RGBBay3.setPosition(0);
            }
            GamePad2();
            if (AbortShooting == 0) {
                SpindexerPositon(Bay2Shooting);
                Shoot();
                Bay2Status = 0;
                RGBBay2.setPosition(0);
            }
            GamePad2();
            if (AbortShooting == 0) {
                Spindexer.setPosition(Bay1Shooting);
                SpindexerPositon(Bay1Shooting);
                Shoot();
                Bay1Status = 0;
                RGBBay1.setPosition(0);
            }
        }
        Bay1Sensing();
        Bay2Sensing();
        Bay3Sensing();
        GamePad2();
        if (AbortShooting == 0) {
            if (Bay1Status > 0.5) {
                SpindexerPositon(Bay1Shooting);
                Shoot();
                Bay1Status = 0;
                RGBBay1.setPosition(0);
            }
        }
        GamePad2();
        if (AbortShooting == 0) {
            if (Bay2Status > 0.5) {
                SpindexerPositon(Bay2Shooting);
                Shoot();
                Bay2Status = 0;
                RGBBay2.setPosition(0);
            }
        }
        GamePad2();
        if (AbortShooting == 0) {
            if (Bay3Status > 0.5) {
                SpindexerPositon(Bay3Shooting);
                Shoot();
                Bay3Status = 0;
                RGBBay3.setPosition(0);
            }
        }
        Spindexer.setPosition(Bay1IntakeLoading);
    }

    /**
     * Describe this function...
     */
    private void SpindexerPositon(double SpindexerDesired) {
        double SpindexerEncoderTolerance;
        double SpindexerDesiredEncoder;

        ServoTimer.reset();
        Spindexer.setPosition(SpindexerDesired);
        SpindexerEncoderTolerance = 0.02 / 2;
        SpindexerDesiredEncoder = 2.9696 * SpindexerDesired + 0.1641;
        while (!(Math.abs(SpindexerEncoder.getVoltage() - SpindexerDesiredEncoder) <= SpindexerEncoderTolerance)) {
            Driving();
            if (ServoTimer.milliseconds() > SpindexerTransitDelayTime) {
                break;
            }
        }
    }

    /**
     * Describe this function...
     */
    private void ShootGreen() {
        if (Bay1Status == 2) {
            SpindexerPositon(Bay1Shooting);
            Shoot();
            Bay1Status = 0;
            RGBBay1.setPosition(0);
        } else if (Bay2Status == 2) {
            SpindexerPositon(Bay2Shooting);
            Shoot();
            Bay2Status = 0;
            RGBBay2.setPosition(0);
        } else if (Bay3Status == 2) {
            SpindexerPositon(Bay3Shooting);
            Shoot();
            Bay3Status = 0;
            RGBBay3.setPosition(0);
        }
    }

    /**
     * Describe this function...
     */
    private void ShootPurple() {
        if (Bay1Status == 3) {
            SpindexerPositon(Bay1Shooting);
            Shoot();
            Bay1Status = 0;
            RGBBay1.setPosition(0);
        } else if (Bay2Status == 3) {
            SpindexerPositon(Bay2Shooting);
            Shoot();
            Bay2Status = 0;
            RGBBay2.setPosition(0);
        } else if (Bay3Status == 3) {
            SpindexerPositon(Bay3Shooting);
            Shoot();
            Bay3Status = 0;
            RGBBay3.setPosition(0);
        }
    }

    /**
     * Describe this function...
     */
    private void AutoTrackAprilTag() {
        if (result.getBotposeAvgDist() > 0.5) {
            ShooterRPM = 500.14 * result.getBotposeAvgDist() + 2292.7 + ShooterRPMOffset;
            HoodAngle.setPosition(0.5475 * Math.pow(result.getBotposeAvgDist(), -0.54));
            RGB_VisionAcquired.setPosition(0.5);
        } else {
            ShooterRPM = 3875;
            HoodAngle.setPosition(0.35);
            RGB_VisionAcquired.setPosition(0);
        }
    }
}
