package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

import edu.wpi.first.wpilibj.util.Color;

public class SpinBay {
    private final Servo led;

    private Spindexer.BayState state;
    private ColorSensorResult reading;
    private final ReadColorSensorThread thread;

    public SpinBay(String colorSensorName, String ledName) {
        thread = new ReadColorSensorThread(colorSensorName);
        reading = new ColorSensorResult();
        led = Robot.opMode.hardwareMap.get(Servo.class, ledName);
        reading = new ColorSensorResult();
        state = Spindexer.BayState.None;
        thread.start();
    }

    public void periodic() {
        reading = thread.getResult();
        //do 1 sensor read for the color
        if(reading.dist < 2) {
            var sensedState = Robot.spindexer.matchColor(getColor());

            if (sensedState == Spindexer.BayState.Green || sensedState == Spindexer.BayState.Purple) {
                state = sensedState;
            } else if (state == Spindexer.BayState.None || state == Spindexer.BayState.Something) {
                state = Spindexer.BayState.Something;
            }
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
        reading = new ColorSensorResult();
        thread.resetResult();
    }

    private static class ColorSensorResult{
        public double dist;
        public Color color;
        public double loopTimeMs;

        public ColorSensorResult() {
            dist = 20;
            color = Color.kBlack;
            loopTimeMs = -1;
        }
    }
    private static class ReadColorSensorThread extends Thread {
        private ColorSensor colorSensor;
        private DistanceSensor distSensor;
        private ColorSensorResult threadResult;

        public ReadColorSensorThread(String colorSensorName) {
            colorSensor = Robot.opMode.hardwareMap.get(ColorSensor.class, colorSensorName);
            distSensor = (DistanceSensor) colorSensor;
            threadResult = new ColorSensorResult();
        }

        public ColorSensorResult getResult() {
            return threadResult;
        }

        public void resetResult() {
            threadResult = new ColorSensorResult();
        }

        // moved to a separate thread because the color sensor sometimes lags
        public void run() {
            //noinspection InfiniteLoopStatement
            while (true) {
                var localResult = new ColorSensorResult();
                long startTime = System.nanoTime();
                localResult.dist = distSensor.getDistance(DistanceUnit.CM);
                localResult.color = new Color(colorSensor.red()/4096., colorSensor.green()/4096., colorSensor.blue()/4096.);
                localResult.loopTimeMs = (System.nanoTime() - startTime) / 1000000.;
                threadResult = localResult;

                //let the system catch up with other work after a read
                Thread.yield();
            }
        }
    }
}
