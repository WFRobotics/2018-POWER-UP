package org.wfrobotics.reuse.subsystems;

import org.wfrobotics.reuse.utilities.Reportable;
import org.wfrobotics.reuse.utilities.Testable;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * STEM Alliance enhanced {@link Subsystem}
 * @author STEM Alliance of Fargo Moorhead
 */
public abstract class EnhancedSubsystem extends Subsystem implements Reportable, Testable
{
    /**
     * Update <b>before</b> {@link Scheduler} is run.
     * Allows periodic {@link EnhancedSubsystem} behavior <b>regardless</b> of active command.
     */
    public abstract void cacheSensors(boolean isDisabled);

    // We don't use these
    @Override
    public void addChild(String name, Sendable child) { }
    @Override
    public void addChild(Sendable child) { }
    @Override
    public void initSendable(SendableBuilder builder) { }
}
