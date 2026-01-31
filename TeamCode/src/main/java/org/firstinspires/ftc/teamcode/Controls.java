package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.button.Trigger;

public class Controls {
    public double getDriveForward() {
        double speed = Robot.opMode.gamepad1.left_stick_y;
        if (!Robot.IsRed) {
            speed = speed * -1;
        }
        return speed;
    }

    public double getDriveRight() {
        double speed = Robot.opMode.gamepad1.left_stick_x;
        if (!Robot.IsRed) {
            speed = speed * -1;
        }
        return speed;
    }

    public double getDriveTurn() {
        return -Robot.opMode.gamepad1.right_stick_x;
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

    public Trigger resetFieldOriented() {
        return new Trigger() {
            @Override
            public boolean get() {
                return Robot.opMode.gamepad1.left_stick_button;
            }
        };
    }
}
