package org.wfrobotics.reuse.subsystems.background;

/** Get highly regular calls to your fast service routine.
 *  Good for time critical, period-sensitive adjustments.
 *  In the time between one iteration of Commands, multiple calls to update handlers will occur.
 *  Ex: Software PID refresh, leader-follower error correction, low latency polling, etc.
 *  Note: Call must be thread-safe. Keep your service routine brief. */
public interface BackgroundUpdate
{
    /**
     * Called <b>once</b> when <b>entering</b> Autonomous or Teleop
     * @param isAutonomous Mode we are <b><i>just</i></b> about to enter
     */
    public void onStartUpdates(boolean isAutonomous);
    /** Fast background worker calls this periodically */
    public void onBackgroundUpdate();
    /**
     * Called <b>once</b> when <b>entering</b> Disabled
     */
    public void onStopUpdates();
}
