package org.wfrobotics.reuse.hardware.sensors;

import org.wfrobotics.reuse.math.HerdAngle;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

// TODO should we use callback?

public class GyroNavx implements Gyro
{
    private final AHRS navxMXP;
    private double zeroAngle = 0;

    public GyroNavx()
    {
        navxMXP = new AHRS(SPI.Port.kMXP, (byte) 200); // TODO increased update rate: Is that okay?
        navxMXP.zeroYaw();
    }

    /** Reset yaw with offset. Future gyro output yaw will be relative to (yaw right now less the offset).
     *  Ex: If offset 90 degrees, use (current - 90 degrees) as "zero".
     *      Thus getYaw() will initially return (0 - (0 - 90)) = +90 degrees.
     *      This example could correct the gyro in an autonomous mode where the robot starts facing "to the right".
     */
    public void zeroYaw(double robotRelativeAngleAsZero)
    {
        zeroAngle = new HerdAngle(navxMXP.getYaw() - robotRelativeAngleAsZero).getAngle();
    }

    /** Yaw relative to when last zeroed */
    public double getAngle()
    {
        return new HerdAngle(navxMXP.getYaw() - zeroAngle).getAngle();  // Ex: If we rotated two degrees clockwise from where yaw was zeroed, yaw = ((zero + 2) - zero) = +2 degrees
    }

    /** Reset yaw. Future gyro output yaw will be relative to the yaw right now. */
    public void zeroYaw()
    {
        zeroAngle = new HerdAngle(navxMXP.getYaw()).getAngle();
    }

    public boolean isOk()
    {
        boolean result = navxMXP.isConnected();

        System.out.println(String.format("Navx gyro %s connected", (result) ? "is" : "is not"));
        return result;
    }

    public void reset()
    {
        navxMXP.reset();
    }

    public double getRate()
    {
        return navxMXP.getRate();
    }
}
