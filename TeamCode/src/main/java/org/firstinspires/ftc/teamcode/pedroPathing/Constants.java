package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class Constants {
    public static FollowerConstants followerConstantsComp = new FollowerConstants()
            .mass(13.38)  //in KG?, 29.5 lbs on 3/7/26
            .forwardZeroPowerAcceleration(-42.3166)
            .lateralZeroPowerAcceleration(-35.1215);

    public static PinpointConstants localizerConstantsComp = new PinpointConstants()
            .forwardPodY(-5)     //forward pod 3.75, -5
            .strafePodX(-2.25)   //strafe  pod -2.25, -5
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    public static FollowerConstants followerConstantsProg = new FollowerConstants()
            .mass(15)  //in KG?
            .forwardZeroPowerAcceleration(-42.3166)
            .lateralZeroPowerAcceleration(-35.1215);

    public static PinpointConstants localizerConstantsProg = new PinpointConstants()
            .forwardPodY(-4.6875)  //forward pod 4, -4.6875
            .strafePodX(-2.5625)   //strafe  pod -2.5625, -5.125
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(56.9842)
            .yVelocity(46.3710);

    public static Follower createFollower(HardwareMap hardwareMap) {
        if(Robot.RobotType == Robot.RobotTypeEnum.Competition) {
            return new FollowerBuilder(followerConstantsComp, hardwareMap)
                    .pathConstraints(pathConstraints)
                    .pinpointLocalizer(localizerConstantsComp)  //must be before mecanum!
                    .mecanumDrivetrain(driveConstants)
                    .build();
        } else {
            return new FollowerBuilder(followerConstantsProg, hardwareMap)
                    .pathConstraints(pathConstraints)
                    .pinpointLocalizer(localizerConstantsProg)  //must be before mecanum!
                    .mecanumDrivetrain(driveConstants)
                    .build();
        }
    }
}