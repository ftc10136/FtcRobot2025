package org.livoniawarriors;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class TimerEvent extends CommandBase {
    Timer timer;
    boolean running;

    public TimerEvent() {
        running = false;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.resetTimer();
        running = true;
    }

    @Override
    public void execute() {
        timer.resetTimer();
        running = true;
    }

    @Override
    public boolean isFinished() {
        //Exiting quickly, but the timer still runs
        return true;
    }

    public BooleanSupplier onElapsed(double timeoutSec) {
        return new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                if (running && timer.getElapsedTimeSeconds() > timeoutSec) {
                    return true;
                } else {
                    return false;
                }
            }
        };
    }

    public Trigger timerEvent(double timeoutSec) {
        return new Trigger(onElapsed(timeoutSec));
    }
}
