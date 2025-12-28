package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.button.Trigger;

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

    public Trigger hpLoadActive() {
        return new Trigger() {
            @Override
            public boolean get() {
                return Robot.opMode.gamepad1.x || Robot.opMode.gamepad2.x;
            }
        };
    }

    public Trigger floorLoadActive() {
        return new Trigger() {
            @Override
            public boolean get() {
                return Robot.opMode.gamepad1.a || Robot.opMode.gamepad2.a;
            }
        };
    }
}
