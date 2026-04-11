package org.livoniawarriors;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot;

public class LoggerCommandTimer {
    TelemetryPacket packet;
    long markedTime;
    long startTime;
    String name;

    public LoggerCommandTimer(String Name) {
        name = Name;
        packet = new TelemetryPacket();
        Robot.logPacket(packet);
    }

    public Command startLog() {
        return new InstantCommand(()-> {
            markedTime = System.nanoTime();
            startTime = markedTime;
        });
    }

    public Command addEntry(String key) {
        return new CommandBase() {
            @Override
            public void execute() {
                long newTime = System.nanoTime();
                double deltaTime = (newTime - markedTime) / 1_000_000.;
                packet.put("TimingMs/" + name + "/" + key, deltaTime);
                markedTime = newTime;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public Command finishLog(String key) {
        return new CommandBase() {
            @Override
            public void execute() {
                long newTime = System.nanoTime();
                double deltaTime = (newTime - markedTime) / 1_000_000.;
                packet.put("TimingMs/" + name + "/" + key, deltaTime);
                packet.put("TimingMs/" + name + "/TotalTime", (newTime - startTime) / 1_000_000.);
                Robot.logPacket(packet);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }
}
