package org.wfrobotics.reuse.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import org.wfrobotics.reuse.config.TalonConfig.Gains;
import org.wfrobotics.reuse.config.TankConfig;
import org.wfrobotics.reuse.hardware.TalonChecker;
import org.wfrobotics.reuse.hardware.TalonFactory;
import org.wfrobotics.reuse.hardware.sensors.Gyro;
import org.wfrobotics.reuse.hardware.sensors.GyroPigeon;
import org.wfrobotics.reuse.math.geometry.Kinematics;
import org.wfrobotics.reuse.math.geometry.Pose2d;
import org.wfrobotics.reuse.math.geometry.Pose2dWithCurvature;
import org.wfrobotics.reuse.math.geometry.Twist2d;
import org.wfrobotics.reuse.subsystems.EnhancedSubsystem;
import org.wfrobotics.reuse.subsystems.background.BackgroundUpdate;
import org.wfrobotics.reuse.subsystems.background.BackgroundUpdater;
import org.wfrobotics.reuse.subsystems.control.Path;
import org.wfrobotics.reuse.subsystems.control.PathFollower;
import org.wfrobotics.reuse.subsystems.control.PathFollower.Parameters;
import org.wfrobotics.reuse.subsystems.control2018.DriveMotionPlanner;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.TrajectoryIterator;
import org.wfrobotics.reuse.subsystems.control2018.trajectory.timing.TimedState;
import org.wfrobotics.reuse.utilities.ReflectingCSVWriter;
import org.wfrobotics.robot.RobotState;
import org.wfrobotics.robot.config.RobotConfig;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** @author STEM Alliance of Fargo Moorhead */
public final class TankSubsystem extends EnhancedSubsystem
{
    private enum TANK_MODE
    {
        OPEN_LOOP,
        LOCK_SETPOINT,
        DISTANCE,
        TURN,
        PATH,
        PATH_2018,
    }

    private final BackgroundUpdate updater = new BackgroundUpdate()
    {
        public void onStartUpdates(boolean isAutonomous)
        {
            synchronized (TankSubsystem.this)
            {
                final boolean brake = isAutonomous || kBrakeOpenLoop;  // Mutually exclusive, so this works

                setBrake(brake);  // Set once, avoids talon brief disables from otherwise setting in commands
                leftMaster.configNeutralDeadband((isAutonomous) ? kDeadbandOpenLoop : 0.0, 0);
                rightMaster.configNeutralDeadband((isAutonomous) ? kDeadbandOpenLoop : 0.0, 0);
            }
        }

        public void onBackgroundUpdate()
        {
            synchronized (TankSubsystem.this)
            {
                switch (controlMode)
                {
                    case OPEN_LOOP:
                        break;
                    case LOCK_SETPOINT:
                        break;
                    case TURN:
                        updateTurnToHeading();
                        break;
                    case DISTANCE:
                        updateDriveDistance();
                        break;
                    case PATH:
                        if (pathFollower != null)
                        {
                            updatePath();
                            if (kTuning)  // TODO Remake log each path to print headers each time
                            {
                                log.add(pathFollower.getDebug());
                            }
                        }
                        break;
                    case PATH_2018:
                        updatePathFollower();
                        break;
                    default:
                        System.out.println("Drive Updater: Bad control mode");
                        break;
                }
            }
        }

        public void onStopUpdates()
        {
            leftMaster.neutralOutput();  // Bypass lazy talon
            rightMaster.neutralOutput();
            log.flush();
            log2.flush();
        }
    };

    private static final boolean kBrakeOpenLoop;
    private static final double kDeadbandOpenLoop;
    private static final int kSlotMotionMagicControl, kSlotTurnControl, kSlotVelocityControl;
    private static final boolean kTuning;
    private static final double kDVelocity;
    private static final Parameters kPathConfig;

    private static TankSubsystem instance = new TankSubsystem();
    private final RobotState state = RobotState.getInstance();
    private final ReflectingCSVWriter<PathFollower.DebugOutput> log = new ReflectingCSVWriter<PathFollower.DebugOutput>(PathFollower.DebugOutput.class);  // or "/home/lvuser/PATH-FOLLOWER-LOGS.csv",
    private final SteeringController steeringDriveDistance;
    private PathFollower pathFollower;
    private CachedIO cachedIO = new CachedIO();

    private final TalonSRX leftMaster, rightMaster;
    private final ArrayList<BaseMotorController> followers = new ArrayList<BaseMotorController>();
    private final Gyro gyro;

    private TANK_MODE controlMode = TANK_MODE.OPEN_LOOP;
    private double targetDistance, targetHeading, targetDistanceL, targetDistanceR = 0.0;
    private boolean brakeModeEnabled;

    // New stuff
    private DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner();
    private boolean mOverrideTrajectory = false;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<DriveMotionPlanner.Output> log2 = new ReflectingCSVWriter<DriveMotionPlanner.Output>(DriveMotionPlanner.Output.class);

    static
    {
        final TankConfig config = RobotConfig.getInstance().getTankConfig();
        kBrakeOpenLoop = config.OPEN_LOOP_BRAKE;
        kDeadbandOpenLoop = config.OPEN_LOOP_DEADBAND;
        kSlotMotionMagicControl = config.CLOSED_LOOP.gainsByName("Motion Magic").kSlot;
        kSlotVelocityControl = config.CLOSED_LOOP.gainsByName("Velocity").kSlot;
        kSlotTurnControl = config.CLOSED_LOOP.gainsByName("Turn").kSlot;
        kDVelocity = config.CLOSED_LOOP.gainsByName("Velocity").kD;
        kTuning = config.TUNING;
        kPathConfig = config.getPathConfig();
    }

    private TankSubsystem()
    {
        final TankConfig config = RobotConfig.getInstance().getTankConfig();

        List<TalonSRX> masters = TalonFactory.makeClosedLoopTalon(config.CLOSED_LOOP);
        for (TalonSRX master : masters)
        {
            master.setControlFramePeriod(ControlFrame.Control_3_General, 5);
            master.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
            master.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100);
            master.configVelocityMeasurementWindow(1, 100);
            master.configOpenloopRamp(config.OPEN_LOOP_RAMP, 100);
            master.configNeutralDeadband(kDeadbandOpenLoop, 100);
            TalonFactory.configFastErrorReporting(master, kTuning);
        }
        leftMaster = masters.get(0);
        rightMaster = masters.get(1);

        for (BaseMotorController follower : TalonFactory.makeFollowers(leftMaster, config.CLOSED_LOOP.masters.get(0)))
        {
            followers.add(follower);
        }
        for (BaseMotorController follower : TalonFactory.makeFollowers(rightMaster, config.CLOSED_LOOP.masters.get(1)))
        {
            followers.add(follower);
        }

        gyro = config.getGyroHardware();
        if (gyro instanceof GyroPigeon)
        {
            rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 100);
        }
        steeringDriveDistance = new SteeringController(config.STEERING_DRIVE_DISTANCE_P, config.STEERING_DRIVE_DISTANCE_I);
        BackgroundUpdater.getInstance().register(updater);

        zeroEncoders();
        brakeModeEnabled = true;  // Opposite to force initial setBrake() to succeed
        setBrake(false);
        setGyro(0.0);
    }

    public static TankSubsystem getInstance()
    {
        return instance;
    }

    protected void initDefaultCommand()
    {
        setDefaultCommand(RobotConfig.getInstance().getTankConfig().getTeleopCommand());
    }

    public void cacheSensors(boolean isDisabled)
    {
        cachedIO.positionTicksL = leftMaster.getSelectedSensorPosition(0);
        cachedIO.positionTicksR = rightMaster.getSelectedSensorPosition(0);
        cachedIO.velocityTicksPer100msL = leftMaster.getSelectedSensorVelocity(0);
        cachedIO.velocityTicksPer100msR = rightMaster.getSelectedSensorVelocity(0);
        cachedIO.headingDegrees = gyro.getAngle();

        state.updateRobotHeading(getGryo());

        if (isDisabled)
        {
            zeroEncoders();
            setGyro(0.0);
            state.resetDriveState(Timer.getFPGATimestamp(), new Pose2d());
        }
    }

    public void reportState()
    {
        SmartDashboard.putString("Drive", getCurrentCommandName());
        SmartDashboard.putNumber("Drive_Error_L", leftMaster.getClosedLoopError(0));
        SmartDashboard.putNumber("Drive_Error_R", rightMaster.getClosedLoopError(0));
        SmartDashboard.putNumber("Drive_Position_L", cachedIO.positionTicksL);
        SmartDashboard.putNumber("Drive_Position_R", cachedIO.positionTicksR);
        SmartDashboard.putNumber("Drive_Velocity_Raw", getVelocityNativeL());
        log.write();
    }

    public synchronized void driveOpenLoop(double left, double right)
    {
        controlMode = TANK_MODE.OPEN_LOOP;
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    public synchronized void turnToHeading(double targetHeading)
    {
        setPIDSlot(kSlotTurnControl);
        this.targetHeading = targetHeading;
        controlMode = TANK_MODE.TURN;
        setTurnSetpoint(targetHeading);  // Extra cycle
    }

    public synchronized void driveDistance(double inchesForward)
    {
        setPIDSlot(kSlotMotionMagicControl);
        targetDistance = inchesForward;
        targetHeading = getGryo();
        steeringDriveDistance.reset(Timer.getFPGATimestamp());
        targetDistanceL = targetDistance + getDistanceInchesL();
        targetDistanceR = targetDistance + getDistanceInchesR();
        state.resetDistanceDriven();
        controlMode = TANK_MODE.DISTANCE;
        updateDriveDistance();  // Extra cycle
    }

    public synchronized void drivePath(Path path, boolean reversed)
    {
        state.resetDistanceDriven();
        pathFollower = new PathFollower(path, reversed, kPathConfig);
        setPIDSlot(kSlotVelocityControl);
        controlMode = TANK_MODE.PATH;
    }

    public synchronized void setBrake(boolean enable)
    {
        if (brakeModeEnabled != enable)
        {
            final NeutralMode mode = (enable) ? NeutralMode.Brake : NeutralMode.Coast;
            leftMaster.setNeutralMode(mode);
            rightMaster.setNeutralMode(mode);
            for (BaseMotorController follower : followers)
            {
                follower.setNeutralMode(mode);
            }
            brakeModeEnabled = enable;
        }
    }

    public synchronized void setGyro(double angle)
    {
        gyro.zeroYaw(angle);
        cachedIO.headingDegrees = angle;
        state.updateRobotHeading(angle);
    }

    /** Changes the rate the robot can accelerate. Potentially useful to prevent tipping, etc. */
    public synchronized void setOpenLoopRampRate(double secondsNeutralToFullThrottle)
    {
        leftMaster.configOpenloopRamp(secondsNeutralToFullThrottle, 0);
        rightMaster.configOpenloopRamp(secondsNeutralToFullThrottle, 0);
        for (BaseMotorController follower : followers)
        {
            follower.configOpenloopRamp(secondsNeutralToFullThrottle, 0);
        }
    }

    public synchronized void zeroEncoders()
    {
        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);
        cachedIO = new CachedIO();
        mPeriodicIO = new PeriodicIO();
    }

    public double getDistanceInchesL()
    {
        return TankMaths.ticksToInches(cachedIO.positionTicksL);
    }

    public double getDistanceInchesR()
    {
        return TankMaths.ticksToInches(cachedIO.positionTicksR);
    }

    public double getGryo()
    {
        return cachedIO.headingDegrees;
    }

    public double getVelocityNativeL()
    {
        return cachedIO.velocityTicksPer100msL;
    }

    public double getVelocityNativeR()
    {
        return cachedIO.velocityTicksPer100msR;
    }

    public double getVeloctiyInchesPerSecondL()
    {
        return TankMaths.ticksToInchesPerSecond(getVelocityNativeL());
    }

    public double getVeloctiyInchesPerSecondR()
    {
        return TankMaths.ticksToInchesPerSecond(getVelocityNativeR());
    }

    public synchronized boolean onTarget()
    {
        return controlMode == TANK_MODE.LOCK_SETPOINT;
    }

    private void updateTurnToHeading()
    {
        final double heading = getGryo();
        final boolean closeEnough = Math.abs(heading - targetHeading) < .25;
        final boolean slowEnough = true;  // TODO Add this criteria?

        setTurnSetpoint((closeEnough && slowEnough) ? heading : targetHeading);
        if (closeEnough && slowEnough)
        {
            controlMode = TANK_MODE.LOCK_SETPOINT;
        }
    }

    private void updateDriveDistance()
    {
        final double now = Timer.getFPGATimestamp();
        final boolean doneSteering = Math.abs(state.getDistanceDriven() - targetDistance) < 9.0;
        final double adjust = steeringDriveDistance.correctHeading(now, targetHeading, getGryo(), doneSteering);

        leftMaster.set(ControlMode.MotionMagic, targetDistanceL, DemandType.ArbitraryFeedForward, adjust);
        rightMaster.set(ControlMode.MotionMagic, targetDistanceR, DemandType.ArbitraryFeedForward, -adjust);
    }

    private void updatePath()
    {
        final double now = Timer.getFPGATimestamp();
        final Twist2d command = pathFollower.update(now, state.getLatestFieldToVehicle().getValue(), state.getDistanceDriven(), state.getPredictedVelocity().dx);

        if (!pathFollower.isFinished())
        {
            final Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);  // Inches/s
            setVelocitySetpoint(setpoint.left, setpoint.right, 0.0, 0.0);
        }
        else
        {
            setVelocitySetpoint(0.0, 0.0, 0.0, 0.0);
            controlMode = TANK_MODE.LOCK_SETPOINT;
        }
    }

    private void setTurnSetpoint(double desiredHeading)
    {
        final double relativeAngleOption0 = desiredHeading - getGryo();
        final double invert = -Math.signum(relativeAngleOption0);
        final double relativeAngleOption1 = relativeAngleOption0 + (invert * 360);
        final double relativeDegrees = (Math.abs(relativeAngleOption0) <= Math.abs(relativeAngleOption1)) ? relativeAngleOption0 : relativeAngleOption1;
        final double radialInches = TankMaths.inchesOfWheelTurning(relativeDegrees);
        final double radialTicks = TankMaths.inchesToTicks(radialInches);

        leftMaster.set(ControlMode.MotionMagic, radialTicks + cachedIO.positionTicksL);
        rightMaster.set(ControlMode.MotionMagic, -radialTicks + cachedIO.positionTicksR);
    }

    private void setPIDSlot(int slot)
    {
        leftMaster.selectProfileSlot(slot, 0);
        rightMaster.selectProfileSlot(slot, 0);
    }

    private void setVelocitySetpoint(double inchesPerSecondL, double inchesPerSecondR, double feedforwardL, double feedforwardR)
    {
        final double nativeL = TankMaths.inchesPerSecondToTicks(inchesPerSecondL);
        final double nativeR = TankMaths.inchesPerSecondToTicks(inchesPerSecondR);
        final double max_desired = Math.max(Math.abs(nativeL), Math.abs(nativeR));
        final double scale = max_desired > TankMaths.kVelocityMax ? TankMaths.kVelocityMax / max_desired : 1.0;

        //        System.out.format("%7.2f, %7.2f, %7.2f, %7.2f\n", nativeL, nativeR, nativeL * scale, nativeR * scale);
        leftMaster.set(ControlMode.Velocity, nativeL * scale, DemandType.ArbitraryFeedForward, feedforwardL + kDVelocity * mPeriodicIO.left_accel / 1023.0);
        rightMaster.set(ControlMode.Velocity, nativeR * scale, DemandType.ArbitraryFeedForward, feedforwardR + kDVelocity * mPeriodicIO.right_accel / 1023.0);
    }

    public TestReport runFunctionalTest()
    {
        final TankConfig config = RobotConfig.getInstance().getTankConfig();
        TestReport report = new TestReport();

        report.add(getDefaultCommand().doesRequire(this), "Default command requires Subsystem");
        report.add(TalonChecker.checkFirmware(leftMaster));
        report.add(TalonChecker.checkFirmware(rightMaster));
        for (BaseMotorController follower : followers)
        {
            report.add(TalonChecker.checkFirmware(follower));
        }
        report.add(TalonChecker.checkEncoder(leftMaster));
        report.add(TalonChecker.checkEncoder(rightMaster));
        report.add(TalonChecker.checkFrameRates(leftMaster));
        report.add(TalonChecker.checkFrameRates(rightMaster));
        report.add(TalonChecker.checkSensorPhase(0.20, leftMaster, rightMaster));
        report.add(gyro.isOk());
        report.add(config.CLOSED_LOOP.gains.size() == 3, "Num Gains");
        for (Gains g : config.CLOSED_LOOP.gains)
        {
            System.out.println(g);
        }

        return report;
    }

    private class CachedIO
    {
        public int positionTicksL;
        public int positionTicksR;
        public int velocityTicksPer100msL;
        public int velocityTicksPer100msR;
        public double headingDegrees;
    }

    // New stuff

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory)
    {
        if (mMotionPlanner != null)
        {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            controlMode = TANK_MODE.PATH_2018;
        }
    }

    private void updatePathFollower()
    {
        if (controlMode == TANK_MODE.PATH_2018)
        {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            log2.add(output);

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory)
            {
                final double left = radiansPerSecondToTicksPer100ms(output.left_velocity);
                final double right = radiansPerSecondToTicksPer100ms(output.right_velocity);
                final double leftFeedForward = output.left_feedforward_voltage / 12.0;
                final double rightFeedForward = output.right_feedforward_voltage / 12.0;
                setVelocitySetpoint(left, right, leftFeedForward, rightFeedForward);

                // TODO Add back in
                //                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                //                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
                mPeriodicIO.left_accel = 0.0;
                mPeriodicIO.right_accel = 0.0;
            }
            else
            {
                setVelocitySetpoint(0.0, 0.0, 0.0, 0.0);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }

            // TODO Is this modification okay?
            if (mMotionPlanner.isDone() || mOverrideTrajectory)
            {
                System.out.println("Trajectory Done");
                controlMode = TANK_MODE.LOCK_SETPOINT;
            }
        } else
        {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s)
    {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    public static class PeriodicIO
    {
        // INPUTS
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
}