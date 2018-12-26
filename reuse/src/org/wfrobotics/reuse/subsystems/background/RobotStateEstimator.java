package org.wfrobotics.reuse.subsystems.background;

import org.wfrobotics.reuse.math.geometry.Kinematics;
import org.wfrobotics.reuse.math.geometry.Rotation2d;
import org.wfrobotics.reuse.math.geometry.Twist2d;
import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.robot.RobotState;

import edu.wpi.first.wpilibj.Timer;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 * @author Team 254
 */
public final class RobotStateEstimator implements BackgroundUpdate
{
    static RobotStateEstimator instance_ = new RobotStateEstimator();
    private final RobotState robot_state_ = RobotState.getInstance();
    private final TankSubsystem drive_ = TankSubsystem.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;

    private RobotStateEstimator() {}

    public static RobotStateEstimator getInstance()
    {
        return instance_;
    }

    public synchronized void onStartUpdates(boolean isAutonomous)
    {
        left_encoder_prev_distance_ = drive_.getDistanceInchesL();
        right_encoder_prev_distance_ = drive_.getDistanceInchesR();
    }

    public synchronized void onBackgroundUpdate()
    {
        final double left_distance = drive_.getDistanceInchesL();
        final double right_distance = drive_.getDistanceInchesR();
        final Rotation2d gyro_angle = Rotation2d.fromDegrees(-drive_.getGryo());
        final Twist2d odometry_velocity = robot_state_.getRobotOdometry(
                                        left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getVeloctiyInchesPerSecondL(),
                                        drive_.getVeloctiyInchesPerSecondR());
        robot_state_.addRobotObservation(Timer.getFPGATimestamp(), odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

    public void onStopUpdates()
    {

    }
}
