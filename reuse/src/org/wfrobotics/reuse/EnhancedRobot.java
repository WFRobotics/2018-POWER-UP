package org.wfrobotics.reuse;

import org.wfrobotics.reuse.config.AutoFactory;
import org.wfrobotics.reuse.config.AutoRunner;
import org.wfrobotics.reuse.hardware.AutoTune;
import org.wfrobotics.reuse.subsystems.SubsystemRunner;
import org.wfrobotics.reuse.subsystems.background.BackgroundUpdater;
import org.wfrobotics.reuse.subsystems.background.RobotStateEstimator;
import org.wfrobotics.reuse.subsystems.drive.TankSubsystem;
import org.wfrobotics.reuse.utilities.ConsoleLogger;
import org.wfrobotics.reuse.utilities.DashboardView;
import org.wfrobotics.robot.Robot;
import org.wfrobotics.robot.RobotState;
import org.wfrobotics.robot.config.IO;
import org.wfrobotics.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * Base robot for STEM Alliance FRC teams
 * @author STEM Alliance of Fargo Moorhead
 * */
public abstract class EnhancedRobot extends IterativeRobot
{
    protected final BackgroundUpdater backgroundUpdater = BackgroundUpdater.getInstance();
    protected final SubsystemRunner subsystems = SubsystemRunner.getInstance();
    protected final AutoRunner autos = AutoRunner.getInstance();

    /** Register subsystems specific to this robot with {@link SubsystemRunner}. Reuse subsystems (ex: {@link TankSubsystem}) are automatically registered. */
    protected abstract void registerRobotSpecific();

    @Override
    public void robotInit()
    {
        registerRobotSpecific();
        subsystems.register(SuperStructure.getInstance());
        subsystems.register(TankSubsystem.getInstance());
        subsystems.registerReporter(RobotState.getInstance());
        subsystems.registerReporter(backgroundUpdater);
        subsystems.registerReporter(ConsoleLogger.getInstance());
        subsystems.registerReporter(AutoTune.getInstance());
        subsystems.registerTest(AutoFactory.getInstance());
        BackgroundUpdater.getInstance().register(RobotStateEstimator.getInstance());

        IO.getInstance().assignButtons();                // Initialize Buttons after subsystems
        AutoFactory.getInstance().onSelectionChanged();  // Set default auto mode
        DashboardView.startPerformanceCamera();
    }

    @Override
    public void autonomousInit()
    {
        Robot.leds.setForAuto(m_ds.getAlliance());
        backgroundUpdater.start(true);

        autos.startMode();
    }

    @Override
    public void teleopInit()
    {
        autos.stopMode();

        Robot.leds.setForTeleop();
        backgroundUpdater.start(false);
    }

    @Override
    public void disabledInit()
    {
        autos.stopMode();
        backgroundUpdater.stop();
    }

    @Override
    public void testInit()
    {
        autos.stopMode();

        Robot.leds.signalHumanPlayer();  // Pit safety
        boolean result = subsystems.runFunctionalTests();
        Robot.leds.signalFunctionalTestResult(result);
        Timer.delay(3.0);  // Display result
    }

    @Override
    public void autonomousPeriodic()
    {
        subsystems.update();
    }

    @Override
    public void teleopPeriodic()
    {
        subsystems.update();
    }

    @Override
    public void disabledPeriodic()
    {
        subsystems.update();
    }

    @Override
    public void testPeriodic()
    {

    }
}
