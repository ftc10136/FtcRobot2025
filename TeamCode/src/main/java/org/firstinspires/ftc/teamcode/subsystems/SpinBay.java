package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

import edu.wpi.first.wpilibj.util.Color;

public class SpinBay {
    private final Servo led;

    private Spindexer.BayState state;
    public ColorSensorResult reading;
    //private final ReadColorSensorThread thread;
    private double readTime;
    int count;
    int bayNum;

    private RevColorSensorV3 colorSensor;
    private DistanceSensor distSensor;


    public SpinBay(String colorSensorName, String ledName, int bayNum) {
        this.bayNum = bayNum - 1;
        //thread = new ReadColorSensorThread(colorSensorName);
        reading = new ColorSensorResult();
        led = Robot.opMode.hardwareMap.get(Servo.class, ledName);
        reading = new ColorSensorResult();
        state = Spindexer.BayState.None;
        readTime = 0;
        count = 0;

        colorSensor = Robot.opMode.hardwareMap.get(RevColorSensorV3.class, colorSensorName);
        distSensor = (DistanceSensor) colorSensor;

        //thread.start();
    }

    public void periodic() {
        count++;
        /*if(reading.count == newReading.count) {
            return;
        }*/
        //slowing down reads to just 1 bay per periodic call
        if((count % 3) == bayNum) {
            reading = getResult();
        } else {
            return;
        }
        //do 1 sensor read for the color
        if(reading.dist < 2) {
            readTime += reading.loopTimeMs;
            //we want 50ms of reads before we trust a reading
            if(readTime < Robot.RobotConfig.BALL_BAY_TIME_MS) {
                return;
            }
            var sensedState = Robot.spindexer.matchColor(getColor());

            if (sensedState == Spindexer.BayState.Green || sensedState == Spindexer.BayState.Purple) {
                state = sensedState;
            } else if (state == Spindexer.BayState.None || state == Spindexer.BayState.Something) {
                state = Spindexer.BayState.Something;
            }
        } else {
            readTime = 0;
        }
        led.setPosition(Robot.spindexer.getLedColor(state));
    }

    public Spindexer.BayState getState() {
        return state;
    }

    public Color getColor() {
        if(reading.color == null) {
            return Color.kBlack;
        }
        return reading.color;
    }

    public double getDist() {
        return reading.dist;
    }

    public void resetBayState() {
        state = Spindexer.BayState.None;
        readTime = 0;
    }

    public ColorSensorResult getResult() {
        var localResult = new ColorSensorResult();
        long startTime = System.nanoTime();
        var reading = colorSensor.getNormalizedColors();
        localResult.dist = distSensor.getDistance(DistanceUnit.CM);
        localResult.color = new Color(reading.red * 16, reading.green * 16, reading.blue * 16);
        localResult.loopTimeMs = (System.nanoTime() - startTime) / 1000000.;
        localResult.count = count;
        count++;
        return localResult;
    }

    public static class ColorSensorResult{
        public double dist;
        public Color color;
        public double loopTimeMs;
        public int count;

        public ColorSensorResult() {
            dist = 20;
            color = Color.kBlack;
            loopTimeMs = 0;
            count = 0;
        }
    }
    /*
    private class ReadColorSensorThread extends Thread {
        private ColorSensorResult threadResult;

        public ReadColorSensorThread(String colorSensorName) {
            threadResult = new ColorSensorResult();
        }

        public ColorSensorResult getThreadResult() {
            return threadResult;
        }

        // moved to a separate thread because the color sensor sometimes lags
        public void run() {
            //noinspection InfiniteLoopStatement
            while (true) {
                threadResult = getResult();
                //let the system catch up with other work after a read
                Thread.yield();
            }
        }
    }
     */
}
