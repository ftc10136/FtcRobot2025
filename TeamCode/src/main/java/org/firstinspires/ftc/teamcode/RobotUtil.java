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

    /**
     * Returns modulus of input.
     *
     * @param input Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }
}
