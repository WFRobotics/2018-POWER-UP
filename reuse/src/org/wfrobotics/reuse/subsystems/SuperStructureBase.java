package org.wfrobotics.reuse.subsystems;

import org.wfrobotics.reuse.hardware.lowleveldriver.Compressor;

/**
 * Parent {@link EnhancedSubsystem} for everything on the structure that connects the subsystems
 * @author STEM Alliance of Fargo Moorhead
 */
public abstract class SuperStructureBase extends EnhancedSubsystem
{
    protected final Compressor compressor;

    protected SuperStructureBase()
    {
        this.setName("Super Structure");
        compressor = new Compressor();  // CAN address zero, does not conflict with PDP also zero since it's a different CAN device type
        compressor.start();
    }

    public void cacheSensors(boolean isDisabled)
    {

    }

    public void reportState()
    {

    }

    /** Enable or force disable the {@link Compressor} closed loop which <i>automatically</i> runs the {@link Compressor} when pressure if low */
    public void setCompressor(boolean enable)
    {
        if(enable)
        {
            compressor.start();
        }
        else
        {
            compressor.stop();
        }
    }

    public TestReport runFunctionalTest()
    {
        TestReport report = new TestReport();

        // Sticky faults persist across power cycles and cause PCM LED to blink orange (and therefore PDP to blink orange)
        boolean currentTooHighFault = compressor.getCompressorCurrentTooHighStickyFault();  // Max continuous 12V / 17A
        boolean notConnectedFault = compressor.getCompressorNotConnectedStickyFault();
        boolean shortedFault = compressor.getCompressorShortedStickyFault();
        compressor.clearAllPCMStickyFaults();

        System.out.println(String.format("Compressor %s connected", (notConnectedFault) ? "not" : ""));
        System.out.println(String.format("Compressor current %s", (currentTooHighFault) ? "too high" : "okay"));
        System.out.println(String.format("Compressor %s shorted", (shortedFault) ? "" : "not"));

        report.add(!currentTooHighFault);
        report.add(!notConnectedFault);
        report.add(!shortedFault);

        if (!report.allPassed())
        {
            System.out.println("WARNING: Make sure your compressor is set to CAN address zero in Internet Explorer");
        }

        // Status LED:
        //     Green = No faults
        //     Orange = Sticky fault
        //     Red = Active fault, only reset on PCM reboot, not auto-cleared
        //         No COM - Strobe (Put in middle of CAN bus or add ternimating resistor if on end of bus)
        //         Compressor fault - Long strobe
        //         Solenoid fault - Slow blink index
        //     Orange + Red = Damanged hardware
        //     Orange + Green = Stuck in bootloader
        //     Off = No power

        // TODO Try HardwareHLUsageReporting

        return report;
    }
}
