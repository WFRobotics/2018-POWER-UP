package org.wfrobotics.reuse.commands.drive;

import org.wfrobotics.reuse.math.control.CheesyDriveHelper;
import org.wfrobotics.reuse.math.control.CheesyDriveHelper.DriveSignal;
import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.robot.config.IO;

import edu.wpi.first.wpilibj.command.Command;

/** Default {@link TankSubsystem} {@link Command} in teleop */
public class DriveCheesy extends Command
{
    protected final TankSubsystem drive = TankSubsystem.getInstance();
    protected final IO io = IO.getInstance();
    protected static final CheesyDriveHelper helper = new CheesyDriveHelper();

    public DriveCheesy()
    {
        requires(drive);
    }

    protected void execute()
    {
        final DriveSignal s = helper.cheesyDrive(io.getThrottle(), io.getTurn(), io.getDriveQuickTurn(), false);
        drive.driveOpenLoop(s.getLeft(), s.getRight());
    }

    protected boolean isFinished()
    {
        return false;
    }
}
