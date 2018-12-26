package org.wfrobotics.reuse.hardware.sensors;

import org.wfrobotics.reuse.math.HerdAngle;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class GyroPigeon implements Gyro
{
    private final PigeonIMU pigeon;

    public GyroPigeon(TalonSRX talon)
    {
        pigeon = new PigeonIMU(talon);
    }

    public double getAngle()
    {
        final double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return new HerdAngle(-ypr[0]).getAngle();
    }

    public void zeroYaw()
    {
        zeroYaw(0);
    }

    public void zeroYaw(double robotRelativeAngleAsZero)
    {
        pigeon.setYaw(robotRelativeAngleAsZero, 10);
        pigeon.setFusedHeading(0.0, 10);
        pigeon.setAccumZAngle(0, 10);
    }

    public boolean isOk()
    {
        boolean result = pigeon.getState() == PigeonIMU.PigeonState.Ready;

        System.out.println(String.format("Pigeon gyro %s connected", (result) ? "is" : "is not"));
        return result;
    }
}
