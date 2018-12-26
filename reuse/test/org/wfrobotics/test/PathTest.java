package org.wfrobotics.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.wfrobotics.reuse.math.geometry.Translation2d;
import org.wfrobotics.reuse.math.motion.MotionState;
import org.wfrobotics.reuse.subsystems.control.Path;
import org.wfrobotics.reuse.subsystems.control.PathSegment;

public class PathTest
{
    public static final MotionState stopped = new MotionState(0, 0, 0, 0);

    public PathSegment getSegment()
    {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 17.0;
        double y1 = 0.0;
        double velocity = 6.25;
        double endSpeed = 0.0;
        return new PathSegment(x0, y0, x1, y1, velocity, stopped, endSpeed);
    }

    @Test
    public void testSegment()
    {
        double x1 = 17.0;
        PathSegment ps = getSegment();

        double delta = 5.0;
        double remaining = ps.getRemainingDistance(new Translation2d(delta, 1.0));
        assertTrue(remaining > x1 - delta);
        System.out.println(remaining);
        assertEquals(ps.getRemainingDistance(new Translation2d(delta, 0.0)), x1 - delta, 0.0);
    }

    @Test
    public void testPath()
    {
        PathSegment ps = getSegment();
        Path path = new Path();

        path.addSegment(ps);
        System.out.println(path);
        System.out.println(path.getSegmentRemainingDist(new Translation2d(5.0, 1.0)));
    }
}
