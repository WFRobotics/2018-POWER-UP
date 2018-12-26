package org.wfrobotics.reuse.subsystems.background;

import java.util.ArrayList;
import java.util.List;

import org.wfrobotics.reuse.utilities.CrashTrackingRunnable;
import org.wfrobotics.reuse.utilities.Reportable;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Runs Robot fast background updates - Calls registered background update event handles at regular, fast rate */
public final class BackgroundUpdater implements Reportable
{
    /** Rate at which registered fast background updates are serviced. For comparison, Commands are updated every ~.02 seconds when the driver station communicates with the Robot. */
    public final double kPeriodSeconds = 0.005;
    private static BackgroundUpdater instance = null;
    private final Notifier notifier_;
    private final List<BackgroundUpdate> loops_;
    private final Object taskRunningLock_;

    private boolean running = false;
    private double prevTime = 0.0;
    private double dt = 0.0;

    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable()
    {
        @Override
        public void runCrashTracked()
        {
            synchronized (taskRunningLock_)
            {
                if (running)
                {
                    double now = Timer.getFPGATimestamp();
                    dt = now - prevTime;
                    prevTime = now;

                    for (BackgroundUpdate loop : loops_)
                    {
                        loop.onBackgroundUpdate();
                    }
                }
            }
        }
    };

    public BackgroundUpdater()
    {
        notifier_ = new Notifier(runnable_);
        running = false;
        loops_ = new ArrayList<>();
        taskRunningLock_ = new Object();  // Mutex between updater and runner
    }

    public static BackgroundUpdater getInstance()
    {
        if (instance == null)
        {
            instance = new BackgroundUpdater();
        }
        return instance;
    }

    /** Schedule an additional fast background service loop */
    public synchronized void register(BackgroundUpdate loop)
    {
        synchronized (taskRunningLock_)
        {
            loops_.add(loop);
        }
    }

    /** Start running fast background service routines */
    public synchronized void start(boolean isAutonomous)
    {
        if (!running)
        {
            System.out.println("Starting loops");
            synchronized (taskRunningLock_)
            {
                prevTime = Timer.getFPGATimestamp();
                for (BackgroundUpdate loop : loops_)
                {
                    loop.onStartUpdates(isAutonomous);
                }
                running = true;
            }
            notifier_.startPeriodic(kPeriodSeconds);
        }
    }

    /** Stop running fast background service routines */
    public synchronized void stop()
    {
        if (running)
        {
            System.out.println("Stopping loops");
            notifier_.stop();
            dt = 0.0;
            synchronized (taskRunningLock_)
            {
                running = false;
                for (BackgroundUpdate loop : loops_)
                {
                    loop.onStopUpdates();
                }
            }
        }
    }

    public void reportState()
    {
        SmartDashboard.putNumber("Background Period", dt * 1000.0);
    }
}
