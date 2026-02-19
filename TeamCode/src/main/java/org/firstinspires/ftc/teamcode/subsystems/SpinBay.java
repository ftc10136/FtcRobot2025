package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

import edu.wpi.first.wpilibj.util.Color;

public class SpinBay {
    private final ColorSensor colorSensor;
    private final DistanceSensor distSensor;
    private final Servo led;

    private Spindexer.BayState state;
    private Color color;
    private double dist;

    public SpinBay(String colorSensorName, String ledName) {
        colorSensor = Robot.opMode.hardwareMap.get(ColorSensor.class, colorSensorName);
        distSensor = (DistanceSensor) colorSensor;
        led = Robot.opMode.hardwareMap.get(Servo.class, ledName);
        color = Color.kBlack;
        state = Spindexer.BayState.None;
    }

    public void periodic() {
        //do 1 sensor read for the color
        dist = distSensor.getDistance(DistanceUnit.CM);
        if(dist < 2) {
            color = new Color(colorSensor.red()/4096., colorSensor.green()/4096., colorSensor.blue()/4096.);
            var sensedState = Robot.spindexer.matchColor(color);

            if (sensedState == Spindexer.BayState.Green || sensedState == Spindexer.BayState.Purple) {
                state = sensedState;
            } else if (state == Spindexer.BayState.None || state == Spindexer.BayState.Something) {
                state = Spindexer.BayState.None;
            }
        }
        led.setPosition(Robot.spindexer.getLedColor(state));
    }

    public Spindexer.BayState getState() {
        return state;
    }

    public Color getColor() {
        return color;
    }

    public double getDist() {
        return dist;
    }

    public void resetBayState() {
        state = Spindexer.BayState.None;
    }
}
