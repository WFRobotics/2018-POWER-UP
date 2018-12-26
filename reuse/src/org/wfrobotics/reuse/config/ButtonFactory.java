package org.wfrobotics.reuse.config;

import org.wfrobotics.reuse.config.Xbox.AXIS;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Assigns {@link Command}s to run when {@link Button}s are pressed
 * @author STEM Alliance of Fargo Moorhead
 */
public abstract class ButtonFactory
{
    public enum TRIGGER
    {
        CANCEL_WHEN_PRESSED,
        TOGGLE_WHEN_PRESSED,
        WHEN_PRESSED,
        WHEN_RELEASED,
        WHILE_HELD;
    }

    public static Button makeButton(Xbox provider, Xbox.BUTTON button, TRIGGER when, Command action)
    {
        return setTrigger(new XboxButton(provider, button), when, action);
    }

    public static Button makeButton(HerdJoystick provider, HerdJoystick.BUTTON button, TRIGGER when, Command action)
    {
        return setTrigger(new JoystickButton(provider, button), when, action);
    }

    public static Button makeButton(Xbox provider, Xbox.AXIS axis, double thresholdOn, TRIGGER when, Command action)
    {
        return setTrigger(new XboxAxisButton(provider, axis, thresholdOn), when, action);
    }


    public static Button makeButton(Xbox provider, AXIS axis, double thresholdPos, double thresholdNeg, TRIGGER when, Command action)
    {
        return setTrigger(new XboxAxisButton(provider, axis, thresholdPos, thresholdNeg), when, action);
    }

    public static Button makeButton(Xbox provider, Xbox.DPAD button, TRIGGER when, Command action)
    {
        return setTrigger(new XboxDpadButton(provider, button), when, action);
    }

    public static Button makeAnyDpadButton(Xbox provider, TRIGGER when, Command action)
    {
        return setTrigger(new AnyDpadButton(provider), when, action);
    }

    private static Button setTrigger(Button b, TRIGGER when, Command action)
    {
        switch (when)
        {
            case CANCEL_WHEN_PRESSED:
                b.cancelWhenPressed(action);
                break;
            case TOGGLE_WHEN_PRESSED:
                b.toggleWhenPressed(action);
                break;
            case WHEN_PRESSED:
                b.whenPressed(action);
                break;
            case WHEN_RELEASED:
                b.whenReleased(action);
                break;
            case WHILE_HELD:
            default:
                b.whileHeld(action);
                break;
        }
        return b;
    }

    private static class XboxButton extends Button
    {
        Xbox hardware;
        Xbox.BUTTON button;

        public XboxButton(Xbox hardware, Xbox.BUTTON button)
        {
            this.hardware = hardware;
            this.button = button;
        }

        public boolean get()
        {
            return hardware.getButtonPressed(button);
        }
    }

    private static class JoystickButton extends Button
    {
        HerdJoystick hardware;
        HerdJoystick.BUTTON button;

        public JoystickButton(HerdJoystick hardware, HerdJoystick.BUTTON button)
        {
            this.hardware = hardware;
            this.button = button;
        }

        public boolean get()
        {
            return hardware.getButtonPressed(button);
        }
    }

    private static class XboxDpadButton extends Button
    {
        Xbox hardware;
        Xbox.DPAD direction;

        public XboxDpadButton(Xbox hardware, Xbox.DPAD direction)
        {
            this.hardware = hardware;
            this.direction = direction;
        }

        public boolean get()
        {
            return hardware.getButtonPressed(direction);
        }
    }

    private static class XboxAxisButton extends Button
    {
        Xbox hardware;
        Xbox.AXIS axis;
        double limitPos;
        boolean checkNeg;
        double limitNeg;

        public XboxAxisButton(Xbox hardware, Xbox.AXIS axis, double thresholdOn)
        {
            this.hardware = hardware;
            this.axis = axis;
            limitPos = thresholdOn;
            checkNeg = false;
        }

        public XboxAxisButton(Xbox hardware, Xbox.AXIS axis, double thresholdPos, double thresholdNeg)
        {
            this.hardware = hardware;
            this.axis = axis;
            limitPos = thresholdPos;
            limitNeg = thresholdNeg;
            checkNeg = true;
        }

        public boolean get()
        {
            if(checkNeg)
            {
                return (hardware.getAxis(axis) > limitPos || hardware.getAxis(axis) < limitNeg);
            }
            return hardware.getAxis(axis) > limitPos;
        }
    }

    private static class AnyDpadButton extends Button
    {
        Xbox hardware;

        public AnyDpadButton(Xbox hardware)
        {
            this.hardware = hardware;
        }

        public boolean get()
        {
            return hardware.getDpad() != Xbox.DPAD.NONE.get();
        }
    }

    /** Two buttons for use with Command based robot */
    public class DualButton extends Button
    {
        Button button1;
        Button button2;

        public DualButton(Button button1, Button button2)
        {
            this.button1 = button1;
            this.button2 = button2;
        }

        public boolean get()
        {
            return button1.get() && button2.get();
        }
    }
}
