package org.wfrobotics.robot.config;

import org.wfrobotics.reuse.commands.wrapper.AutoMode;
import org.wfrobotics.reuse.config.AutoFactory.DelaySelect;
import org.wfrobotics.reuse.config.AutoFactory.ModeSafetyOff;
import org.wfrobotics.reuse.config.AutoFactory.ModeSelectBase;
import org.wfrobotics.reuse.config.AutoSelection;
import org.wfrobotics.robot.auto.ModeCenter;
import org.wfrobotics.robot.auto.ModeOppisitScalse;
import org.wfrobotics.robot.auto.ModeScale;
import org.wfrobotics.robot.auto.ModeSide;

/** Configuration of {@link AutoMode} can be run in autonomous mode*/
public abstract class Auto
{
    public static ModeSelect modes = new ModeSelect();
    public static DelaySelect delays = new DelaySelect();
    public static PositionSelect positions = new PositionSelect();

    public static class ModeSelect extends ModeSelectBase
    {
        protected AutoMode[] options()
        {
            return new AutoMode[] {
                new ModeSafetyOff(),
                new ModeScale(),
                new ModeOppisitScalse(),
                new ModeCenter(),
                new ModeSide(),
            };
        }
    }

    public static enum POSITION { LEFT, CENTER, RIGHT, };

    public static class PositionSelect extends AutoSelection<POSITION>
    {
        protected POSITION[] options()
        {
            return new POSITION[] {POSITION.LEFT, POSITION.CENTER, POSITION.RIGHT};
        }

        protected void apply() { }
    }
}