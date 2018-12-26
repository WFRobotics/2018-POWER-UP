package org.wfrobotics.reuse.hardware.sensors;

public interface Gyro
{
    public double getAngle();
    public boolean isOk();
    public void zeroYaw();
    public void zeroYaw(double robotRelativeAngleAsZero);
}
