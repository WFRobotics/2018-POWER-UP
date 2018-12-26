package org.wfrobotics.reuse.commands.wrapper;

import org.wfrobotics.reuse.utilities.ConsoleLogger;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/** <i>Impromptu</i> way of having multiple commands <b>both finish</b> before next {@link Command} is run */
public class SynchronizedCommand extends CommandGroup
{
    /**
     * Run each {@link Command} in an impromptu {@link CommandGroup}, requiring them <b>all to finish</b> before the {@link SynchronizedCommand} is finished
     *
     * <p>Note: You can make your {@link SynchronizedCommand} look <b>super freckin pretty</b> by passing it an array
     * <p>ex: new SynchronizedCommand(new Command[] { &ltyour commands each on a newline&gt });
     *
     * @param Each {@link Command} that must finish before {@link SynchronizedCommand} is finsihed
     */
    public SynchronizedCommand(Command... commands)
    {
        if (commands.length < 2)
        {
            ConsoleLogger.warning(String.format("%s called with zero parallel commands", this.getClass().getSimpleName()));
        }

        for (int index = 1; index < commands.length; index++)
        {
            this.addParallel(commands[index]);
        }
        this.addSequential(commands[0]);
    }
}