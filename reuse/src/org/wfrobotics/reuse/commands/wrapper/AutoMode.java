package org.wfrobotics.reuse.commands.wrapper;

import org.wfrobotics.reuse.config.AutoFactory;
import org.wfrobotics.reuse.config.AutoRunner;
import org.wfrobotics.reuse.utilities.ConsoleLogger;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Parent class for autonomous modes. Ran by {@link AutoRunner}. Built by {@link AutoFactory}.
 * @author STEM Alliance of Fargo Moorhead
 */
public abstract class AutoMode extends CommandGroup
{
    protected int startingDelay = 0;

    @Override
    protected void end()
    {
        ConsoleLogger.info(String.format("%s ran in %.2f seconds", this, timeSinceInitialized()));
    }

    public void setDelay(int initialDelay)
    {
        startingDelay = initialDelay;
    }

    public String toString()
    {
        return String.format("%s", getClass().getSimpleName());
    }
}
