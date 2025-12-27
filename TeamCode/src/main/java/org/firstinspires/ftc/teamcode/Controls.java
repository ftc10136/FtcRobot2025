package org.firstinspires.ftc.teamcode;

public class Controls {
    public double getDriveForward() {
        return Robot.opMode.gamepad1.left_stick_y;
    }

    public double getDriveRight() {
        return Robot.opMode.gamepad1.left_stick_x;
    }

    public double getDriveTurn() {
        return Robot.opMode.gamepad1.right_stick_x;
    }

    public boolean slowModeActive() {
        return Robot.opMode.gamepad1.left_trigger >= 0.2;
    }
}
