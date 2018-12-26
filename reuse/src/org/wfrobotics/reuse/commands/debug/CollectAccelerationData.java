package org.wfrobotics.reuse.commands.debug;

import java.util.List;

import org.wfrobotics.reuse.math.geometry.Util;
import org.wfrobotics.reuse.math.physics.DriveCharacterization;
import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.reuse.utilities.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class CollectAccelerationData extends Command
{
    private static final double kPower = 0.5;
    private static final double kTotalTime = 2.0; //how long to run the test for
    private static final double kVolts = kPower * 12.0;
    private final boolean mTurn;
    private final boolean mReverse;

    private final TankSubsystem mDrive = TankSubsystem.getInstance();
    private final ReflectingCSVWriter<DriveCharacterization.AccelerationDataPoint> mCSVWriter;
    private final List<DriveCharacterization.AccelerationDataPoint> mAccelerationData;

    private double mStartTime = 0.0;
    private double mPrevVelocity = 0.0;
    private double mPrevTime = 0.0;

    public CollectAccelerationData(List<DriveCharacterization.AccelerationDataPoint> data, boolean reverse, boolean turn)
    {
        requires(mDrive);
        mAccelerationData = data;
        mReverse = reverse;
        mTurn = turn;
        mCSVWriter = new ReflectingCSVWriter<>(DriveCharacterization.AccelerationDataPoint.class);
    }

    protected void initialize()
    {
        mDrive.driveOpenLoop((mReverse ? -1.0 : 1.0) * kPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kPower);
        mStartTime = Timer.getFPGATimestamp();
        mPrevTime = mStartTime;
    }

    protected void execute()
    {
        final double currentVelocity = (Math.abs(mDrive.getVelocityNativeL()) + Math.abs(mDrive.getVelocityNativeR())) / 4096.0 * Math.PI * 10;
        final double currentTime = Timer.getFPGATimestamp();

        //don't calculate acceleration until we've populated prevTime and prevVelocity
        if (mPrevTime == mStartTime) {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        //ignore accelerations that are too small
        final double acceleration = (currentVelocity - mPrevVelocity) / (currentTime - mPrevTime);
        if (acceleration < Util.kEpsilon)
        {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        mAccelerationData.add(new DriveCharacterization.AccelerationDataPoint(currentVelocity, kVolts, acceleration));
        mCSVWriter.add(mAccelerationData.get(mAccelerationData.size() - 1));

        mPrevTime = currentTime;
        mPrevVelocity = currentVelocity;
    }

    protected boolean isFinished()
    {
        return Timer.getFPGATimestamp() - mStartTime > kTotalTime;
    }

    protected void end()
    {
        mDrive.driveOpenLoop(0.0, 0.0);
        mCSVWriter.flush();
    }
}
