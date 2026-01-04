package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.livoniawarriors.RobotUtil;

public class HoodAngle extends SubsystemBase {
    private final Servo hoodAngle;
    private final TelemetryPacket packet;
    public HoodAngle() {
        packet = new TelemetryPacket();
        hoodAngle = Robot.opMode.hardwareMap.get(Servo.class, "HoodAngle");
        hoodAngle.scaleRange(0.2, 0.8);
        hoodAngle.setPosition(0.35);
    }

    @Override
    public void periodic() {
        packet.put("HoodAngle/ServoCommand", hoodAngle.getPosition());
        packet.put("HoodAngle/Command", RobotUtil.getCommandName(getCurrentCommand()));
        Robot.logPacket(packet);
    }
}
