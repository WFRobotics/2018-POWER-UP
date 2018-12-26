package org.wfrobotics.reuse.commands.drive;

import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.robot.RobotState;

import edu.wpi.first.wpilibj.command.Command;

/** Turn until reaching the target, assumes target in view at start **/
public class TurnToInViewTarget extends Command
{
    protected final RobotState state = RobotState.getInstance();
    private final TankSubsystem drive = TankSubsystem.getInstance();

    final double tol;

    public TurnToInViewTarget(double tolerance)
    {
        requires(drive);
        tol = tolerance;
    }

    protected void execute()
    {
        final double targetHeading = state.robotHeading + state.viaionAngleError;
        drive.turnToHeading(targetHeading);
    }

    protected boolean isFinished()
    {
        return !state.visionInView || Math.abs(state.viaionAngleError) < tol;
    }

    protected void end()
    {
        drive.driveOpenLoop(0.0, 0.0);
    }
}