package org.wfrobotics.test.vision;

import org.wfrobotics.reuse.subsystems.background.BackgroundUpdate;
import org.wfrobotics.reuse.subsystems.vision.CameraServer.CameraListener;
import org.wfrobotics.reuse.subsystems.vision.CoprocessorData;

/** Allows delaying coprocessor data until time of use, decouples from RobotState slightly */
public class VisionProcessor implements BackgroundUpdate, CameraListener
{
    private static VisionProcessor instance = null;
    private CoprocessorData rx = null;

    public synchronized void Notify(CoprocessorData message)
    {
        rx = message;
    }

    private VisionProcessor() { }

    public static VisionProcessor getInstance()
    {
        if (instance == null)
        {
            instance = new VisionProcessor();
        }
        return instance;
    }


    public void onStartUpdates(boolean isAutonomous)
    {

    }

    public void onBackgroundUpdate()
    {
        CoprocessorData update;
        synchronized (this)
        {
            if (rx == null)
            {
                return;
            }
            update = rx;
            rx = null;
        }
        if (update != null)
        {
            RobotState.getInstance().addVisionUpdate(update.atCameraTimestampMs, update);
        }
    }

    public void onStopUpdates()
    {

    }
}
