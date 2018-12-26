package org.wfrobotics.reuse.config;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Runs the currently selected autonomous mode
 * @author STEM Alliance of Fargo Moorhead
 */
public class AutoRunner
{
    static class SingletonHolder
    {
        static AutoRunner instance = new AutoRunner();
    }

    public static AutoRunner getInstance()
    {
        return SingletonHolder.instance;
    }

    private boolean running = false;
    protected CommandGroup selected = null;

    protected AutoRunner() { }

    /** Set the command currently selected to run when autonomous mode starts */
    public synchronized void register(CommandGroup mode)
    {
        if (!running)
        {
            selected = mode;
        }
    }

    /** Run the selected autonomous mode. Call from Robot.autonomousInit(). */
    public synchronized void startMode()
    {
        if (!running)
        {
            running = true;
            selected.start();
        }
    }

    /** Cancel the currently running autonomous mode. Call from Robot.disabledInit(). */
    public synchronized void stopMode()
    {
        if (running)
        {
            selected.cancel();
            running = false;
        }
        AutoFactory.getInstance().onSelectionChanged();  // Rebuild paths for repeated test runs
    }

    public String toString()
    {
        return (selected != null) ? selected.toString() : "No mode registered";
    }
}
