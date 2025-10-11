package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandRegistry {

    public record CommandWrapper(String name, Command command) {};


    public static void registerCommand(CommandWrapper command) {
        NamedCommands.registerCommand(command.name(), command.command);
    }

    public static void registerAllCommands(CommandWrapper... commands) {
        for (CommandWrapper command : commands) {
            registerCommand(command);
        }
    }
}
