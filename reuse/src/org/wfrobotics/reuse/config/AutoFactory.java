package org.wfrobotics.reuse.config;

import java.util.ArrayList;

import org.wfrobotics.reuse.commands.config.AutoConfig;
import org.wfrobotics.reuse.commands.drive.DriveOff;
import org.wfrobotics.reuse.commands.wrapper.AutoMode;
import org.wfrobotics.reuse.config.AutoSelection.SelectionListener;
import org.wfrobotics.reuse.utilities.Testable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Rebuilds {@link AutoMode} used for Autonomous each time a {@link AutoSelection} is changed
 * @author STEM Alliance of Fargo Moorhead
 */
public class AutoFactory implements SelectionListener, Testable
{
    /** An convenient "Do Nothing" {@link AutoMode} which may be optionally included in your {@link ModeSelectBase} subclass */
    public static class ModeSafetyOff extends AutoMode
    {
        public ModeSafetyOff()
        {
            this.addSequential(new DriveOff());
        }
    }

    /** <b>Subclass this</b> to dynamically <b>set the selected {@link AutoMode}</b> in {@link AutoFactory} and {@link AutoRunner} which will run in autonomous */
    public static abstract class ModeSelectBase extends AutoSelection<AutoMode>
    {
        protected void apply()
        {
            AutoFactory.getInstance().mode = get();
        }
    }

    /** Delay to dynamically add <b>before</b> the selected{@link AutoMode} runs in autonomous */
    public static class DelaySelect extends AutoSelection<Integer>
    {
        protected Integer[] options()
        {
            return new Integer[] {0, 1, 2, 3, 4, 5};
        }

        protected void apply()
        {
            AutoFactory.getInstance().mode.setDelay(get());
        }
    }

    /** Create a Command that increments an {@link AutoSelection} mapped to a Button */
    public AutoConfig makeCommand(AutoSelection<?> selection)
    {
        selection.registerListener(this);
        services.add(selection);
        return new AutoConfig(() -> selection.increment());
    }

    static class SingletonHolder
    {
        static AutoFactory instance = new AutoFactory();
    }

    public static AutoFactory getInstance()
    {
        return SingletonHolder.instance;
    }

    private ArrayList<AutoSelection<?>> services = new ArrayList<AutoSelection<?>>();
    public AutoMode mode;

    public void onSelectionChanged()
    {
        for (AutoSelection<?> s : services)
        {
            s.apply();
        }
        displaySelectedMode();
        AutoRunner.getInstance().register(mode);
    }

    private void displaySelectedMode()
    {
        try
        {
            DriverStation.reportWarning(toString(), false);
            SmartDashboard.putString("Auto: ", toString());
        }
        catch (UnsatisfiedLinkError e)  // Unit test
        {
            System.out.println(this);
        }
        catch (NoClassDefFoundError e)  // Unit test
        {
            System.out.println(this);
        }
    }

    public String toString()
    {
        String joined = "";
        for (int index = 0; index < services.size(); index++)
        {
            joined += services.get(index).toString() + ((index < services.size() - 1) ? ", " : "");
        }
        return joined;
    }

    public TestReport runFunctionalTest()
    {
        TestReport report = new TestReport();

        int numModes = services.get(0).options().length;  // Assume first registered
        System.out.println("Num Modes: " + numModes);
        report.add(services.size() >= 2);  // Has registered: modes, delays, optional game-specifics or robot-specifics
        report.addManualTest("Current Selection: " + this);

        return report;
    }
}
