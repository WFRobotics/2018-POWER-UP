package org.wfrobotics.reuse.subsystems.control2018;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import org.wfrobotics.reuse.config.TankConfig;
import org.wfrobotics.reuse.math.CSVWritable;
import org.wfrobotics.reuse.math.geometry.Pose2d;
import org.wfrobotics.reuse.math.geometry.Pose2dWithCurvature;
import org.wfrobotics.reuse.math.geometry.Rotation2d;
import org.wfrobotics.reuse.math.geometry.Translation2d;
import org.wfrobotics.reuse.math.geometry.Util;
import org.wfrobotics.reuse.subsystems.control2018.physics.DCMotorTransmission;
import org.wfrobotics.reuse.subsystems.control2018.physics.DifferentialDrive;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.DistanceView;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.PurePursuitController;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.Trajectory;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.TrajectoryIterator;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.TrajectorySamplePoint;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.TrajectoryUtil;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.timing.DifferentialDriveDynamicsConstraint;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.timing.TimedState;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.timing.TimingConstraint;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.timing.TimingUtil;
import org.wfrobotics.reuse.subsystems.control2018.util.Units;
import org.wfrobotics.robot.config.RobotConfig;

public class DriveMotionPlanner implements CSVWritable {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);

    private final double kPathLookaheadTime;  // Seconds to look ahead along the path for steering
    private final double kPathMinLookaheadDistance;  // Inches
    private final double kPathKX;  // units/s per unit of error

    public enum FollowerType {
        FEEDFORWARD_ONLY,
        PURE_PURSUIT,
        PID,
        NONLINEAR_FEEDBACK
    }

    //    FollowerType mFollowerType = FollowerType.NONLINEAR_FEEDBACK;  // DRL Not ready to understand this
    FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    final DifferentialDrive mModel;

    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    Output mOutput = new Output();

    DifferentialDrive.ChassisState prev_velocity_ = new DifferentialDrive.ChassisState();
    double mDt = 0.0;

    public DriveMotionPlanner() {
        final TankConfig config = RobotConfig.getInstance().getTankConfig();
        final double kDriveWheelDiameterInches = config.WHEEL_DIAMETER;
        final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
        final double kDriveWheelTrackWidthInches = config.WIDTH;
        final double kTrackScrubFactor = config.SCRUB;
        final double kDriveKv = 0.00001;  // TODO
        final double kDriveKa = 0.000001; // TODO
        final double kDriveVIntercept = 1.055;  // TODO borrowed from 254 2018
        final double kRobotLinearInertia = 60.0;  // kg TODO borrowed from 254 2018
        final double kRobotAngularInertia = 10.0;  // kg m^2 TODO borrowed from 254 2018
        final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO borrowed from 254 2018
        kPathLookaheadTime = 0.4;  // TODO borrowed from 254 2018
        kPathMinLookaheadDistance = 24.0;  // TODO borrowed from 254 2018
        kPathKX = 4.0;  // TODO borrowed from 254 2018

        final DCMotorTransmission transmission = new DCMotorTransmission(
                                        1.0 / kDriveKv,
                                        Units.inches_to_meters(kDriveWheelRadiusInches) * Units.inches_to_meters(kDriveWheelRadiusInches) * kRobotLinearInertia / (2.0 * kDriveKa),
                                        kDriveVIntercept);
        mModel = new DifferentialDrive(
                                        kRobotLinearInertia,
                                        kRobotAngularInertia,
                                        kRobotAngularDrag,
                                        Units.inches_to_meters(kDriveWheelDiameterInches / 2.0),
                                        Units.inches_to_meters(kDriveWheelTrackWidthInches / 2.0 * kTrackScrubFactor),
                                        transmission, transmission
                                        );
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mError = Pose2d.identity();
        mOutput = new Output();
        mLastTime = Double.POSITIVE_INFINITY;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
                                                                          boolean reversed,
                                                                          final List<Pose2d> waypoints,
                                                                          final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                                                                          double max_vel,  // inches/s
                                                                          double max_accel,  // inches/s^2
                                                                          double max_voltage) {
        return generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
                                                                          boolean reversed,
                                                                          final List<Pose2d> waypoints,
                                                                          final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                                                                          double start_vel,
                                                                          double end_vel,
                                                                          double max_vel,  // inches/s
                                                                          double max_accel,  // inches/s^2
                                                                          double max_voltage) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
        // TODO re-architect the spline generator to support reverse.
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(
                                        waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped.add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip), -trajectory
                                                .getState(i).getCurvature(), trajectory.getState(i).getDCurvatureDs()));
            }
            trajectory = new Trajectory<>(flipped);
        }
        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                                        DifferentialDriveDynamicsConstraint<>(mModel, max_voltage);
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory
                                        (reversed, new
                                                                        DistanceView<>(trajectory), kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel);
        return timed_trajectory;
    }

    @Override
    public String toCSV() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(mOutput.left_velocity) + "," + fmt.format(mOutput.right_velocity) + "," + fmt.format
                                        (mOutput.left_feedforward_voltage) + "," + fmt.format(mOutput.right_feedforward_voltage) + "," +
                                        mSetpoint.toCSV();
    }

    public static class Output {
        public Output() {
        }

        public Output(double left_velocity, double right_velocity, double left_accel, double right_accel,
                      double left_feedforward_voltage, double
                      right_feedforward_voltage) {
            this.left_velocity = left_velocity;
            this.right_velocity = right_velocity;
            this.left_accel = left_accel;
            this.right_accel = right_accel;
            this.left_feedforward_voltage = left_feedforward_voltage;
            this.right_feedforward_voltage = right_feedforward_voltage;
        }

        public double left_velocity;  // rad/s
        public double right_velocity;  // rad/s

        public double left_accel;  // rad/s^2
        public double right_accel;  // rad/s^2

        public double left_feedforward_voltage;
        public double right_feedforward_voltage;

        public void flip() {
            double tmp_left_velocity = left_velocity;
            left_velocity = -right_velocity;
            right_velocity = -tmp_left_velocity;

            double tmp_left_accel = left_accel;
            left_accel = -right_accel;
            right_accel = -tmp_left_accel;

            double tmp_left_feedforward = left_feedforward_voltage;
            left_feedforward_voltage = -right_feedforward_voltage;
            right_feedforward_voltage = -tmp_left_feedforward;
        }
    }

    protected Output updatePID(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
        // Feedback on longitudinal error (distance).
        final double kPathKX = 5.0;
        final double kPathKY = 1.0;
        final double kPathKTheta = 5.0;
        adjusted_velocity.linear = dynamics.chassis_velocity.linear + kPathKX * Units.inches_to_meters
                                        (mError.getTranslation().x());
        adjusted_velocity.angular = dynamics.chassis_velocity.angular + dynamics.chassis_velocity.linear * kPathKY *
                                        Units.inches_to_meters(mError.getTranslation().y()) + kPathKTheta * mError.getRotation().getRadians();

        double curvature = adjusted_velocity.angular / adjusted_velocity.linear;
        if (Double.isInfinite(curvature)) {
            adjusted_velocity.linear = 0.0;
            adjusted_velocity.angular = dynamics.chassis_velocity.angular;
        }

        // Compute adjusted left and right wheel velocities.
        final DifferentialDrive.WheelState wheel_velocities = mModel.solveInverseKinematics(adjusted_velocity);
        final double left_voltage = dynamics.voltage.left + (wheel_velocities.left - dynamics.wheel_velocity
                                        .left) / mModel.left_transmission().speed_per_volt();
        final double right_voltage = dynamics.voltage.right + (wheel_velocities.right - dynamics.wheel_velocity
                                        .right) / mModel.right_transmission().speed_per_volt();

        return new Output(wheel_velocities.left, wheel_velocities.right, dynamics.wheel_acceleration.left, dynamics
                                        .wheel_acceleration.right, left_voltage, right_voltage);
    }

    protected Output updatePurePursuit(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        double lookahead_time = kPathLookaheadTime;
        //        final double kLookaheadSearchDt = 0.01;
        final double kLookaheadSearchDt = 0.005;  // TODO DRL backgroundupdater dt different
        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        while (actual_lookahead_distance < kPathMinLookaheadDistance &&
                                        mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }
        if (actual_lookahead_distance < kPathMinLookaheadDistance) {
            lookahead_state = new TimedState<>(new Pose2dWithCurvature(lookahead_state.state()
                                            .getPose().transformBy(Pose2d.fromTranslation(new Translation2d(
                                                                            (mIsReversed ? -1.0 : 1.0) * (kPathMinLookaheadDistance -
                                                                                                            actual_lookahead_distance), 0.0))), 0.0), lookahead_state.t()
                                            , lookahead_state.velocity(), lookahead_state.acceleration());
        }

        DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
        // Feedback on longitudinal error (distance).
        adjusted_velocity.linear = dynamics.chassis_velocity.linear + kPathKX * Units.inches_to_meters
                                        (mError.getTranslation().x());

        // Use pure pursuit to peek ahead along the trajectory and generate a new curvature.
        final PurePursuitController.Arc<Pose2dWithCurvature> arc = new PurePursuitController.Arc<>(current_state,
                                        lookahead_state.state());

        double curvature = 1.0 / Units.inches_to_meters(arc.radius);
        if (Double.isInfinite(curvature)) {
            adjusted_velocity.linear = 0.0;
            adjusted_velocity.angular = dynamics.chassis_velocity.angular;
        } else {
            adjusted_velocity.angular = curvature * dynamics.chassis_velocity.linear;
        }

        dynamics.chassis_velocity = adjusted_velocity;
        dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);
        return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration
                                        .left, dynamics.wheel_acceleration.right, dynamics.voltage.left, dynamics.voltage.right);
    }

    protected Output updateNonlinearFeedback(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        // Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        final double kBeta = 2.0;  // >0.
        final double kZeta = 0.7;  // Damping coefficient, [0, 1].

        // Compute gain parameter.
        final double k = 2.0 * kZeta * Math.sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity
                                        .linear + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular);

        // Compute error components.
        final double angle_error_rads = mError.getRotation().getRadians();
        final double sin_x_over_x = Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ?
                                        1.0 : mError.getRotation().sin() / angle_error_rads;
        final DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState(
                                        dynamics.chassis_velocity.linear * mError.getRotation().cos() +
                                        k * Units.inches_to_meters(mError.getTranslation().x()),
                                        dynamics.chassis_velocity.angular + k * angle_error_rads +
                                        dynamics.chassis_velocity.linear * kBeta * sin_x_over_x * Units.inches_to_meters(mError
                                                                        .getTranslation().y()));

        // Compute adjusted left and right wheel velocities.
        dynamics.chassis_velocity = adjusted_velocity;
        dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);

        dynamics.chassis_acceleration.linear = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.linear - prev_velocity_
                                        .linear) / mDt;
        dynamics.chassis_acceleration.angular = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.angular - prev_velocity_
                                        .angular) / mDt;

        prev_velocity_ = dynamics.chassis_velocity;

        DifferentialDrive.WheelState feedforward_voltages = mModel.solveInverseDynamics(dynamics.chassis_velocity,
                                        dynamics.chassis_acceleration).voltage;

        return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration
                                        .left, dynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);
    }

    public Output update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return new Output();

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(mDt);
        mSetpoint = sample_point.state();

        if (!mCurrentTrajectory.isDone()) {
            // Generate feedforward voltages.
            final double velocity_m = Units.inches_to_meters(mSetpoint.velocity());
            final double curvature_m = Units.meters_to_inches(mSetpoint.state().getCurvature());
            final double dcurvature_ds_m = Units.meters_to_inches(Units.meters_to_inches(mSetpoint.state()
                                            .getDCurvatureDs()));
            final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
            final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
                                            new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m),
                                            new DifferentialDrive.ChassisState(acceleration_m,
                                                                            acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());

            if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
                mOutput = new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics
                                                .wheel_acceleration.left, dynamics.wheel_acceleration.right, dynamics.voltage
                                                .left, dynamics.voltage.right);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                mOutput = updatePurePursuit(dynamics, current_state);
            } else if (mFollowerType == FollowerType.PID) {
                mOutput = updatePID(dynamics, current_state);
            } else if (mFollowerType == FollowerType.NONLINEAR_FEEDBACK) {
                mOutput = updateNonlinearFeedback(dynamics, current_state);
            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new Output();
        }
        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return mSetpoint;
    }
}
