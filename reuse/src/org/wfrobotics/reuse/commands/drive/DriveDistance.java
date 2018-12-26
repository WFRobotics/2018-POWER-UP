package org.wfrobotics.reuse.commands.drive;

import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.robot.RobotState;
import org.wfrobotics.robot.config.IO;

import edu.wpi.first.wpilibj.command.Command;

/** Drive robot forwards/reverse a distance */
public class DriveDistance extends Command
{
    protected final RobotState state = RobotState.getInstance();
    protected final TankSubsystem drive = TankSubsystem.getInstance();
    protected final IO io = IO.getInstance();

    private double settledSamples;  // Allow PID to work, TODO have subsystem latch once reached instead
    protected double desired;
    protected final double tol = .05;

    public DriveDistance(double inchesForward)
    {
        requires(drive);
        desired = inchesForward;
    }

    protected void initialize()
    {
        drive.setBrake(true);
        settledSamples = 0;
        drive.driveDistance(desired);
    }

    protected boolean isFinished()
    {
        final double error = (desired - state.getDistanceDriven()) / desired;

        if (Math.abs(error) < tol)
        {
            settledSamples++;
        }
        else
        {
            settledSamples = 0;
        }
        return settledSamples > 5  || Math.abs(io.getThrottle()) > 0.15;
    }
}
