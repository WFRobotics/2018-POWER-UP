package org.wfrobotics.reuse.subsystems;

import java.util.Optional;

import org.wfrobotics.reuse.config.TalonConfig.ClosedLoopConfig;
import org.wfrobotics.reuse.hardware.LimitSwitch;
import org.wfrobotics.reuse.hardware.LimitSwitch.Limit;
import org.wfrobotics.reuse.hardware.TalonFactory;
import org.wfrobotics.robot.RobotState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * STEM Alliance {@link EnhancedSubsystem} for subsystems which <b>control a mechinism
 * position</b>, where <i>safe</i> positions are <b>constrained to a limited range of motion</b>
 * @author STEM Alliance of Fargo Moorhead
 */
public abstract class PositionBasedSubsystem extends EnhancedSubsystem
{
    /**
     * Config for {@link PositionBasedSubsystem}. Put in getInstance() or Robot Config.
     */
    public static class PositionConfig
    {
        public ClosedLoopConfig kClosedLoop;
        public boolean kHardwareLimitNormallyOpenB;
        public boolean kHardwareLimitNormallyOpenT;
        /** Observed ticks from the encoder (zero) position to the top (full range) position */
        public int kTicksToTop;
        /** Observed real units (inches or degrees) from the encoder (zero) position to the top (full range) position. */
        public double kFullRangeInchesOrDegrees;
        /** A typical value is slightly below zero (ex: -500) */
        public Optional<Integer> kSoftwareLimitB = Optional.empty();
        /** A typical value is kTicksToTop */
        public Optional<Integer> kSoftwareLimitT = Optional.empty();
        /** Tuning the PID(s) */
        public Optional<Boolean> kTuning = Optional.empty();
    }

    protected final double kFullRangeInchesOrDegrees;
    protected final int kTicksToTop;
    protected final double kTicksPerInchOrDegree;
    protected final boolean kTuning;

    protected final RobotState state = RobotState.getInstance();
    protected final TalonSRX master;
    /** Hardware limit switches. The Talon automatically prevents the motor from going to a position beyond these sensors. */
    protected final LimitSwitch limits;
    /** Saved values of our sensors so we don't have to do repeated reads from the hardware layer */
    protected PositionCachedIO cachedIO = new PositionCachedIO();

    protected boolean hasZeroed;

    public PositionBasedSubsystem(PositionConfig config)
    {
        kTicksToTop = config.kTicksToTop;
        kFullRangeInchesOrDegrees = config.kFullRangeInchesOrDegrees;
        kTicksPerInchOrDegree = kTicksToTop / config.kFullRangeInchesOrDegrees;
        kTuning = config.kTuning.orElse(false);

        master = TalonFactory.makeClosedLoopTalon(config.kClosedLoop).get(0);
        master.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 100);
        master.configVelocityMeasurementWindow(1, 100);
        master.configNeutralDeadband(.05, 100);  // Assume open loop on joystick needs small deadband, set in child constructor if a different value is needed
        TalonFactory.configFastErrorReporting(master, kTuning);

        limits =  new LimitSwitch(master, config.kHardwareLimitNormallyOpenB, config.kHardwareLimitNormallyOpenT);
        LimitSwitch.configSoftwareLimitR(master, config.kSoftwareLimitB.orElse(0), config.kSoftwareLimitB.isPresent());
        LimitSwitch.configSoftwareLimitF(master, config.kSoftwareLimitT.orElse(0), config.kSoftwareLimitT.isPresent());

        hasZeroed = false;
    }

    /** Save {@link PositionCachedIO} values, handle Subsystem zeroing */
    public void cacheSensors(boolean isDisabled)
    {
        cachedIO.positionTicks = master.getSelectedSensorPosition(0);
        cachedIO.velocityTicksPer100ms = master.getSelectedSensorVelocity(0);
        cachedIO.limitSwitchB = limits.isSet(Limit.REVERSE);
        cachedIO.limitSwitchT = limits.isSet(Limit.FORWARD);

        zeroIfAtLimit();
    }

    public void reportState()
    {
        SmartDashboard.putNumber(String.format("%s Position", getName()), getPosition());
        SmartDashboard.putNumber(String.format("%s Position Native", getName()), getPositionNative());
        SmartDashboard.putNumber(String.format("%s Velocity Native", getName()), getVelocityNative());
        SmartDashboard.putBoolean(String.format("%s B", getName()), AtHardwareLimitBottom());
        SmartDashboard.putBoolean(String.format("%s T", getName()), AtHardwareLimitTop());
        SmartDashboard.putNumber(String.format("%s Current",getName()), master.getOutputCurrent());
        SmartDashboard.putBoolean(String.format("%s Zeroed",getName()), hasZeroed());
        if (kTuning)
        {
            SmartDashboard.putNumber(String.format("%s Error",getName()), master.getClosedLoopError(0));
        }
    }

    /**
     * Control the {@link PositionBasedSubsystem} with a <b>setpoint, with PID feedback</b>.
     * Override if your Subsystem needs special logic to determine setpoint or gain scheduling.
     * */
    public void setClosedLoop(double positionSetpoint)
    {
        setMotor(ControlMode.MotionMagic, PositionToNative(positionSetpoint));
    }

    /**
     * Control the {@link PositionBasedSubsystem} with the <b>Joystick, without PID feedback</b>.
     * Override if your Subsystem needs special logic to determine percent.
     */
    public void setOpenLoop(double percent)
    {
        setMotor(ControlMode.PercentOutput, percent);
    }

    /**
     *  Check and handle if Subsystem is at a limit.
     *  Override if your Subsystem also zeros when stalled, at top hardware limit, etc.
     */
    public void zeroIfAtLimit()
    {
        if(AtHardwareLimitBottom())
        {
            zeroEncoder();
        }
    }

    public boolean AtHardwareLimitBottom()
    {
        return cachedIO.limitSwitchB;
    }

    public boolean AtHardwareLimitTop()
    {
        return cachedIO.limitSwitchT;
    }

    /**
     * In inches or degrees from encoder.
     * Override if your coordinate system has zero other than encoder (ex: inches from ground).
     */
    public double getPosition()
    {
        return NativeToPosition(getPositionNative());
    }

    /** Encoder has zeroed at any point, aka Subsystem is ready to go! */
    public boolean hasZeroed()
    {
        return hasZeroed;
    }

    /** <b>Set the motor(s)</b>. Apply any <b>feedforward used in both</b> open & closed loop here. */
    protected void setMotor(ControlMode mode, double setpoint)
    {
        final double feedforward = 0.0;

        master.set(mode, setpoint, DemandType.ArbitraryFeedForward, feedforward);
    }

    /** <b>Reset encoder position to be zero</b> and record zeroing has happened: Position is now known */
    protected void zeroEncoder()
    {
        master.setSelectedSensorPosition(0, 0, 0);
        hasZeroed = true;
    }

    /** In encoder units */
    protected double getPositionNative()
    {
        return cachedIO.positionTicks;
    }

    /** In encoder units */
    protected double getVelocityNative()
    {
        return cachedIO.velocityTicksPer100ms;
    }

    /** Unit conversion */
    protected double PositionToNative(double inchesOrDegrees)
    {
        return inchesOrDegrees * kTicksPerInchOrDegree;
    }

    /** Unit conversion */
    protected double NativeToPosition(double ticks)
    {
        return ticks / kTicksPerInchOrDegree;
    }

    protected static class PositionCachedIO
    {
        public int positionTicks;
        public int velocityTicksPer100ms;
        public boolean limitSwitchB;
        public boolean limitSwitchT;
    }
}
