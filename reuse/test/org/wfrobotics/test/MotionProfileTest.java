package org.wfrobotics.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.wfrobotics.reuse.math.motion.MotionProfile;
import org.wfrobotics.reuse.math.motion.MotionProfileConstraints;
import org.wfrobotics.reuse.math.motion.MotionProfileGenerator;
import org.wfrobotics.reuse.math.motion.MotionProfileGoal;
import org.wfrobotics.reuse.math.motion.MotionState;

public class MotionProfileTest
{
    static double dt = 0.005;
    static MotionState start = new MotionState(0.0, 0.0, 0.0, 0.0);

    public static MotionProfile getMotionProfile(double velocity, double acceleration, double length)
    {
        MotionProfileConstraints constraints = new MotionProfileConstraints(velocity, acceleration);
        MotionProfileGoal goal = new MotionProfileGoal(length);
        return MotionProfileGenerator.generateProfile(constraints, goal, start);
    }

    @Test
    public void testMotionProfileParameterDump()
    {
        double velocity = 6.25;
        double acceleration = 5.7117;
        double length = 17.0;
        MotionProfile mp = getMotionProfile(velocity, acceleration, length);

        System.out.println(mp);
        System.out.println("-------Start-------");
        System.out.println(mp.startPos());
        System.out.println(mp.startTime());
        System.out.println(mp.startState());
        System.out.println("--------End--------");
        System.out.println(mp.endPos());
        System.out.println(mp.endTime());
        System.out.println(mp.endState());
        System.out.println("-----Duration------");
        System.out.println(mp.duration());
        System.out.println(mp.length());
        System.out.println(mp.size());
        System.out.println(mp.isValid());
    }

    @Test
    public void testMotionProfileProducesFauxProfile()
    {
        double inches = 17.0 * 12.0;
        double velocity = 6.25 * 12.0;
        double acceleration = 5.7117 * 12.0;
        double length = inches;
        MotionProfile profile = getMotionProfile(velocity, acceleration, length);

        assertTrue(profile.isValid());

        for (double t = 0; t <= profile.endTime(); t += dt)
        {
            boolean notLast = t <= profile.endTime();
            MotionState state = (notLast) ? profile.stateByTime(t).get() : profile.endState();
            int index = (int) (state.t() / dt);

            System.out.format("%d %7.3f %7.1f %7.1f\n", index, t, state.pos(), state.vel());
        }

        int numPoints = (int) Math.floor(profile.endTime() / dt);
        System.out.println(numPoints + " points");

        assertEquals(profile.stateByTime(profile.duration() / 2.0).get().vel(), velocity, velocity * 0.01);
        assertEquals(numPoints, 762, 0);
        assertEquals(profile.endPos(), length, length * 0.01);
    }
}
