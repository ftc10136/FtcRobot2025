package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.Command;

public class RobotUtil {

    public static String getCommandName(Command command) {
        String commandName;
        if (command == null) {
            commandName = "None";
        } else {
            commandName = command.getName();
        }
        return commandName;
    }
}
