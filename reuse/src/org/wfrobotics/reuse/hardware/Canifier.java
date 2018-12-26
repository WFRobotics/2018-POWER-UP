package org.wfrobotics.reuse.hardware;

import org.wfrobotics.reuse.hardware.lowleveldriver.BlinkinPatterns.PatternName;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class Canifier implements LEDs
{
    static class RGB
    {
        public final int r;
        public final int g;
        public final int b;

        public RGB(int r, int g, int b)
        {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    private static final RGB kRed = new RGB(255, 0, 0);
    private static final RGB kBlue = new RGB(0, 0, 255);
    private static final RGB kSignalHuman = kRed;
    private static final RGB kTestSuccess = new RGB(0, 255, 0);
    private static final RGB kTestFail = kRed;

    private final RGB kDriveTeam;
    private final RGB kTeleop;
    private final CANifier hardware;
    private RGB kAlliance = kBlue;
    private boolean isAuto = false;

    public Canifier(int address, RGB teamColor)
    {
        this(address, teamColor, new RGB(255, 255, 255));
    }

    public Canifier(int address, RGB teamColor, RGB driveTeamColor)
    {
        hardware = new CANifier(address);
        kTeleop = teamColor;
        kDriveTeam = driveTeamColor;
    }

    public void off()
    {
        setLEDs(new RGB(0, 0, 0));
    }

    public void signalDriveTeam()
    {
        setLEDs((isAuto) ? kAlliance : kDriveTeam);  // Override in auto
    }

    public void signalHumanPlayer()
    {
        setLEDs(kSignalHuman);
    }

    public void setForAuto(Alliance team)
    {
        kAlliance = (team == Alliance.Red) ? kRed : kBlue;
        isAuto = true;
        useRobotModeColor();
    }

    public void setForTeleop()
    {
        isAuto = false;
        useRobotModeColor();
    }

    public void useRobotModeColor()
    {
        setLEDs((isAuto) ? kAlliance : kTeleop);
    }

    private void setLEDs(RGB color)
    {
        hardware.setLEDOutput(color.r, CANifier.LEDChannel.LEDChannelC);
        hardware.setLEDOutput(color.g, CANifier.LEDChannel.LEDChannelB);
        hardware.setLEDOutput(color.b, CANifier.LEDChannel.LEDChannelA);
    }

    public boolean testRobotSpecificColors()
    {
        double secondsBetweenColors = 2.0;

        signalHumanPlayer();
        Timer.delay(secondsBetweenColors);
        signalDriveTeam();
        Timer.delay(secondsBetweenColors);
        useRobotModeColor();
        Timer.delay(secondsBetweenColors);
        setForAuto(Alliance.Red);
        Timer.delay(secondsBetweenColors);
        setForAuto(Alliance.Blue);
        return true;
    }

    public boolean testScrollAll(PatternName... vectors)
    {
        return false;
    }

    public boolean testScrollAll()
    {
        return false;
    }

    public void signalFunctionalTestResult(boolean testsPassed)
    {
        setLEDs((testsPassed) ? kTestSuccess : kTestFail);
    }
}
