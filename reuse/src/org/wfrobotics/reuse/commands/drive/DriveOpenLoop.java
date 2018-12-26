package org.wfrobotics.reuse.commands.drive;

import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/** Bang bang control. Not a command you usually want outside rare, special situations */
public class DriveOpenLoop extends Command
{
    private final TankSubsystem drive;
    private final double power;

    public DriveOpenLoop(double percentVoltage)
    {
        drive = TankSubsystem.getInstance();
        requires(drive);
        power = percentVoltage;
    }

    protected void initialize()
    {
        drive.setBrake(true);
    }

    protected void execute()
    {
        drive.setOpenLoopRampRate(power);
    }

    protected boolean isFinished()
    {
        return false;
    }
}
