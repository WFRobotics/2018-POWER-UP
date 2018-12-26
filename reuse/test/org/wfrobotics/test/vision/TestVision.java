package org.wfrobotics.test.vision;

import org.wfrobotics.reuse.subsystems.vision.CameraServer;

/** Receives from local corprocessor */
public class TestVision
{
    public static void main(String args[])
    {
        RobotState state = RobotState.getInstance();
        CameraServer cameraServer = CameraServer.getInstance();
        VisionProcessor processor = VisionProcessor.getInstance();

        cameraServer.register(processor);

        state.resetVisionState();

        while(true)
        {
            processor.onBackgroundUpdate();  // Pretend BackgroundUpdater has started
        }
    }
}
