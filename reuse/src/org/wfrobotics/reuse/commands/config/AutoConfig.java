package org.wfrobotics.reuse.commands.config;

import edu.wpi.first.wpilibj.command.InstantCommand;

/** Selects an Auto Mode parameter */
public class AutoConfig extends InstantCommand
{
    protected Runnable selectNext;  // Functional to force constructor call

    public AutoConfig(Runnable selectNextCallback)
    {
        selectNext = selectNextCallback;
        setRunWhenDisabled(true);
    }

    public void initialize()
    {
        selectNext.run();
    }
}
