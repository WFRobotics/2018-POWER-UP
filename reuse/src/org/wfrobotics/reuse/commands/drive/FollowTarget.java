package org.wfrobotics.reuse.commands.drive;

import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.robot.RobotState;

import edu.wpi.first.wpilibj.command.Command;

/** Turn until reaching the target, or get to the expected heading it should be at **/
public class FollowTarget extends Command
{
    protected final RobotState state = RobotState.getInstance();
    protected final TankSubsystem drive = TankSubsystem.getInstance();

    final double tol;
    public FollowTarget(double tol)
    {
        this.tol = tol;
        requires(drive);

    }
    protected void initialize()
    {
        drive.setBrake(true);
        double angle = state.robotHeading + (-1 * state.viaionAngleError);
        drive.turnToHeading(angle);

    }
    protected void execute()
    {

    }

    protected boolean isFinished()
    {
        if (state.visionInView)
        {
            initialize();
            return false;
        }
        return true;
    }
}
