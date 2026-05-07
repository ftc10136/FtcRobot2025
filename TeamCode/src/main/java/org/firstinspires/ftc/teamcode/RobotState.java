package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

public class RobotState {
    private static boolean hasInit = false;
    private static boolean hasRun = false;
    public static Pose robotPose;
    public static double headingZeroRad;
    public static int helidexerOffset;
    public static int helidexerManualOffset;
    public static double turretManualOffset;
    public static int turretRollovers;
    public static double turretContinuousVoltage;
    public static double turretLastSensorVoltage;
    public static int shooterRpmAdjust;

    public static void initState() {
        if(hasInit) {
            return;
        }

        robotPose = new Pose(72,72,Math.PI/2, PedroCoordinates.INSTANCE);
        hasRun = false;
        helidexerOffset = 0;
        helidexerManualOffset = 0;
        turretManualOffset = 0;
        turretRollovers = 0;
        headingZeroRad = 0;
        shooterRpmAdjust = 100;
        //start in middle of sensor range
        turretLastSensorVoltage = 1.5;
        turretContinuousVoltage = turretLastSensorVoltage;

        hasInit = true;
    }

    public static void initRun() {
        if(hasRun) {
            return;
        }

        helidexerOffset = Robot.helidexer.getRawPosition();
        hasRun = true;
    }
}
