package org.wfrobotics.reuse.commands.debug;

import org.wfrobotics.reuse.commands.config.ResetPose;
import org.wfrobotics.reuse.commands.drive.DriveOff;
import org.wfrobotics.reuse.commands.drive.DrivePath;
import org.wfrobotics.reuse.config.PathContainer;

import edu.wpi.first.wpilibj.command.CommandGroup;

/** Use to rapidly test path by putting on a teleop button */
public class TeleopPath extends CommandGroup
{
    public TeleopPath(PathContainer path)
    {
        this(path, 15.0);
    }

    /** Add a timeout to keep your path from ramming anything accidently */
    public TeleopPath(PathContainer path, double timeout)
    {
        addSequential(new ResetPose(path));
        addSequential(new DrivePath(path), timeout);
        addSequential(new DriveOff());  // Observe robot state
    }
}