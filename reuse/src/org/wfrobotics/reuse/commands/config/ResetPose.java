package org.wfrobotics.reuse.commands.config;

import org.wfrobotics.reuse.config.PathContainer;
import org.wfrobotics.reuse.math.geometry.Pose2d;
import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.robot.RobotState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;

/** Set the absolute coordinates the robot is estimated to be at.
 *  This lets the robot know, for example, its starting location on the field. */
public class ResetPose extends InstantCommand
{
    private final RobotState state;
    private final TankSubsystem drive;
    private PathContainer path;

    public ResetPose(PathContainer path)
    {
        state = RobotState.getInstance();
        drive = TankSubsystem.getInstance();
        this.path = path;
    }

    protected void initialize()
    {
        Pose2d pose = path.getStartPose();
        state.resetDriveState(Timer.getFPGATimestamp(), pose);
        drive.setGyro(pose.getRotation().getDegrees());
    }
}
