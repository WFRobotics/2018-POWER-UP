package org.wfrobotics.reuse.commands.wrapper;

import org.wfrobotics.reuse.utilities.ConsoleLogger;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/** <i>Impromptu</i> way of creating a {@link CommandGroup} of {@link Command}s sequentially without declaring a new class */
public class SeriesCommand extends CommandGroup
{
    /**
     * Run each {@link Command} in an impromptu {@link CommandGroup}
     *
     * <p>Note: You can make your {@link SeriesCommand} look <b>super freckin pretty</b> by passing it an array
     * <p>ex: new SeriesCommand(new Command[] { &ltyour commands each on a newline&gt });
     *
     * @param Each {@link Command} that will run sequentially
     */
    public SeriesCommand(Command... commands)
    {
        if (commands.length < 2)
        {
            ConsoleLogger.warning(String.format("%s called with zero parallel commands", this.getClass().getSimpleName()));
        }

        for (int index = 0; index < commands.length; index++)
        {
            this.addSequential(commands[index]);
        }
    }
}