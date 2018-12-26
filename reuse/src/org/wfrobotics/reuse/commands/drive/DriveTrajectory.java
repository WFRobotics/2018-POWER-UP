package org.wfrobotics.reuse.commands.drive;

import org.wfrobotics.reuse.math.geometry.Pose2dWithCurvature;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.TimedView;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.Trajectory;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.TrajectoryIterator;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.timing.TimedState;
import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.robot.RobotState;
import org.wfrobotics.robot.config.IO;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveTrajectory extends Command
{
    private final RobotState state = RobotState.getInstance();
    private final TankSubsystem drive = TankSubsystem.getInstance();
    private final IO io = IO.getInstance();
    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory;
    private final boolean resetPose;

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory)
    {
        this(trajectory, false);
    }

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose)
    {
        requires(drive);
        this.trajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        this.resetPose = resetPose;
    }

    protected void initialize()
    {
        drive.setBrake(true);
        System.out.println("Starting trajectory! (length=" + trajectory.getRemainingProgress() + ")");
        if (resetPose)
        {
            state.resetDriveState(Timer.getFPGATimestamp(), trajectory.getState().state().getPose());
        }
        drive.setTrajectory(trajectory);
    }

    protected boolean isFinished()
    {
        return drive.onTarget() || Math.abs(io.getThrottle()) > 0.15;
    }

    protected void interrupted()
    {
        System.out.println("Path Interrupted");
        drive.setBrake(true);
        drive.driveOpenLoop(0.0, 0.0);
    }
}