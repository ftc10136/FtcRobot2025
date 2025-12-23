package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class JavaTeleop extends OpMode {
    @Override
    public void init() {
        Robot.Init(this);
    }

    @Override
    public void loop() {
        Robot.Periodic();
    }
}
