package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

import edu.wpi.first.wpilibj.util.Color;

public class SpinBay {
    public enum BayState {
        None,
        Something,
        Green,
        Purple
    }
    private final Servo led;

    private BayState state;
    public ColorSensorResult reading;
    //private final ReadColorSensorThread thread;
    private double readTime;
    int count;
    int bayNum;

    private final RevColorSensorV3 colorSensor;
    private final DistanceSensor distSensor;
    private long lastResetTime;

    public SpinBay(String colorSensorName, String ledName, int bayNum) {
        this.bayNum = bayNum - 1;
        //thread = new ReadColorSensorThread(colorSensorName);
        reading = new ColorSensorResult();
        led = Robot.opMode.hardwareMap.get(Servo.class, ledName);
        reading = new ColorSensorResult();
        state = BayState.None;
        readTime = 0;
        count = 0;
        lastResetTime = 0;

        colorSensor = Robot.opMode.hardwareMap.get(RevColorSensorV3.class, colorSensorName);
        distSensor = (DistanceSensor) colorSensor;

        //thread.start();
    }

    public void periodic() {
        if(System.nanoTime() - lastResetTime < 100_000_000) {
            //if we reset the bay, we want to wait 100ms before we read it again
            return;
        }
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
            //var sensedState = Robot.spindexer.matchColor(getColor());

            //if (sensedState == Spindexer.BayState.Green || sensedState == Spindexer.BayState.Purple) {
            //    state = sensedState;
            //} else if (state == Spindexer.BayState.None || state == Spindexer.BayState.Something) {
            //    state = Spindexer.BayState.Something;
            //}
        } else {
            readTime = 0;
        }
        //led.setPosition(Robot.spindexer.getLedColor(state));
    }

    public BayState getState() {
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
        state = BayState.None;
        readTime = 0;
        lastResetTime = System.nanoTime();
    }

    public ColorSensorResult getResult() {
        var localResult = new ColorSensorResult();
        long startTime = System.nanoTime();
        if(state == BayState.None) {
            localResult.dist = distSensor.getDistance(DistanceUnit.CM);
        } else if(state == BayState.Something) {
            localResult.dist = 1;
            var reading = colorSensor.getNormalizedColors();
            localResult.color = new Color(reading.red * 16, reading.green * 16, reading.blue * 16);
        }
        localResult.loopTimeMs = (System.nanoTime() - startTime) / 1000000.;
        localResult.count = count;
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
