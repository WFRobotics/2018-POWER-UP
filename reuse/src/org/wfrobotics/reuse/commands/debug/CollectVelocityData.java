package org.wfrobotics.reuse.commands.debug;

import java.util.List;

import org.wfrobotics.reuse.math.physics.DriveCharacterization;
import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.reuse.utilities.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class CollectVelocityData extends Command
{
    private static final double kMaxPower = 0.25;
    private static final double kRampRate = 0.02;

    private final TankSubsystem mDrive = TankSubsystem.getInstance();
    private final ReflectingCSVWriter<DriveCharacterization.VelocityDataPoint> mCSVWriter;
    private final List<DriveCharacterization.VelocityDataPoint> mVelocityData;
    private final boolean mTurn;
    private final boolean mReverse;

    private boolean isFinished = false;
    private double mStartTime = 0.0;

    public CollectVelocityData(List<DriveCharacterization.VelocityDataPoint> data, boolean reverse, boolean turn)
    {
        requires(mDrive);
        mVelocityData = data;
        mReverse = reverse;
        mTurn = turn;
        mCSVWriter = new ReflectingCSVWriter<>(DriveCharacterization.VelocityDataPoint.class);
    }

    protected void initialize()
    {
        mStartTime = Timer.getFPGATimestamp();
    }

    protected void execute()
    {
        final double percentPower = kRampRate * (Timer.getFPGATimestamp() - mStartTime);

        if (percentPower > kMaxPower)
        {
            isFinished = true;
            return;
        }
        mDrive.driveOpenLoop((mReverse ? -1.0 : 1.0) * percentPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * percentPower);

        double velocity = (Math.abs(mDrive.getVelocityNativeL()) + Math.abs(mDrive.getVelocityNativeR())) / 4096.0 * Math.PI * 10;  //convert velocity to radians per second
        double volts = percentPower * 12.0;
        mVelocityData.add(new DriveCharacterization.VelocityDataPoint(velocity, volts));
        mCSVWriter.add(mVelocityData.get(mVelocityData.size() - 1));
    }

    protected boolean isFinished()
    {
        return isFinished;
    }

    protected void end()
    {
        mDrive.driveOpenLoop(0.0, 0.0);
        mCSVWriter.flush();
    }
}
