package org.wfrobotics.reuse.commands;

import org.wfrobotics.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Alert the Human players to do something. Use in whileheld button
 * @author STEM Alliance of Fargo Moorhead
 */
public class SignalHumanPlayer extends Command
{
    protected void initialize()
    {
        Robot.leds.signalHumanPlayer();
    }

    protected boolean isFinished()
    {
        return false;
    }

    protected void end()
    {
        Robot.leds.useRobotModeColor();
    }
}
