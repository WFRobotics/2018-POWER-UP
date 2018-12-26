package org.wfrobotics.reuse.subsystems;

import java.util.ArrayList;

import org.wfrobotics.reuse.subsystems.background.BackgroundUpdate;
import org.wfrobotics.reuse.subsystems.background.BackgroundUpdater;
import org.wfrobotics.reuse.utilities.ConsoleLogger;
import org.wfrobotics.reuse.utilities.Reportable;
import org.wfrobotics.reuse.utilities.Testable;
import org.wfrobotics.reuse.utilities.Testable.TestReport;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * STEM Alliance enhanced {@link Scheduler}.
 * This class runs one iternation/loop of the robot code: running each {@link EnhancedSubsystem}'s current Command.
 * Each {@link EnhancedSubsystem} is also serviced, calling its Command-independent service methods before and after running each current Command.
 * This happens every ~20ms when the DriverStation updates the IO values.
 * If this usage does not fit your needs, see {@link BackgroundUpdate} or RobotState.
 * @author STEM Alliance of Fargo Moorhead
 */
public final class SubsystemRunner
{
    private static SubsystemRunner instance;
    private final Scheduler scheduler = Scheduler.getInstance();
    private final ArrayList<EnhancedSubsystem> subsystems = new ArrayList<EnhancedSubsystem>();
    private final ArrayList<Reportable> extraReporters = new ArrayList<Reportable>();
    private final ArrayList<Testable> extraTests = new ArrayList<Testable>();

    private SubsystemRunner() {}

    public static SubsystemRunner getInstance()
    {
        if (instance == null)
        {
            instance = new SubsystemRunner();
        }
        return instance;
    }

    public void register(EnhancedSubsystem subsystem)
    {
        subsystems.add(subsystem);
        if (subsystem instanceof BackgroundUpdate)
        {
            BackgroundUpdater.getInstance().register((BackgroundUpdate) subsystem);
        }
    }

    /** {@link Reportable} that will also get called once per Robot code iteration. {@link EnhancedSubsystem} were already registered with {@link register}. */
    public void registerReporter(Reportable extraReporter)
    {
        if (!(extraReporter instanceof EnhancedSubsystem))
        {
            extraReporters.add(extraReporter);
        }
    }

    /** {@link Testable} that will also get called in Robot functional test. {@link EnhancedSubsystem} were already registered with {@link register}. */
    public void registerTest(Testable extraTest)
    {
        if (!(extraTest instanceof EnhancedSubsystem))
        {
            extraTests.add(extraTest);
        }
    }

    public boolean runFunctionalTests()
    {
        TestReport report = new TestReport();

        ConsoleLogger.getInstance().reportState();  // flush
        System.out.println("-------------\nRobot Tests\n-------------");
        Timer.delay(1.0);

        for(int index = 0; index < subsystems.size(); index++)
        {
            report.add(Testable.run(subsystems.get(index)));
        }
        for(int index = 0; index < extraTests.size(); index++)
        {
            report.add(Testable.run(extraTests.get(index)));
        }

        System.out.println("--- Talon Frame Report ---");
        ConsoleLogger.getInstance().reportState();
        System.out.println("--- Test Summary ---");
        System.out.println(String.format("Tested %d Subsystems, %d other components", subsystems.size(), extraTests.size()));
        System.out.println(report);
        System.out.println(String.format("Robot Tests: %s", (report.allPassed()) ? "SUCCESS" : "FAILURE"));

        return report.allPassed();
    }

    public void update()
    {
        updateSensors();
        runCommands();
        reportState();
    }

    private void updateSensors()
    {
        final boolean isDisabled = DriverStation.getInstance().isDisabled();
        for(int index = 0; index < subsystems.size(); index++)
        {
            subsystems.get(index).cacheSensors(isDisabled);
        }
    }

    private void runCommands()
    {
        final double schedulerStart = Timer.getFPGATimestamp();
        scheduler.run();
        SmartDashboard.putNumber("Periodic Time ", Timer.getFPGATimestamp() - schedulerStart);
    }

    private void reportState()
    {
        for(int index = 0; index < subsystems.size(); index++)
        {
            subsystems.get(index).reportState();
        }
        for(int index = 0; index < extraReporters.size(); index++)
        {
            extraReporters.get(index).reportState();
        }
    }
}
