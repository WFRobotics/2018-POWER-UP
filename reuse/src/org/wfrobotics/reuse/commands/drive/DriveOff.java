package org.wfrobotics.reuse.commands.drive;

import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.reuse.utilities.ConsoleLogger;
import org.wfrobotics.robot.config.IO;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/** Safety {@link Command} for {@link TankSubsystem). Toggle or cancel to quit. **/
public class DriveOff extends Command
{
    private final TankSubsystem drive = TankSubsystem.getInstance();
    private final IO io = IO.getInstance();

    public DriveOff()
    {
        requires(drive);
    }

    protected void initialize()
    {
        ConsoleLogger.warning("Drive: Off");
        ConsoleLogger.warning("Match Time Remaining: " + DriverStation.getInstance().getMatchTime());
        drive.setBrake(true);
    }

    protected void execute()
    {
        drive.driveOpenLoop(0.0, 0.0);
    }

    protected boolean isFinished()
    {
        return Math.abs(io.getThrottle()) > 0.15;
    }
}
