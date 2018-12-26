package org.wfrobotics.test;

import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.wfrobotics.reuse.math.HerdAngle;
import org.wfrobotics.reuse.math.HerdVector;

public class HerdAngleTest
{
    @Test
    public void testNoWrap()
    {
        // Constructor - No wrap
        assertTrue(new HerdAngle(10).getAngle() == 10); // "Positive in valid range failed to stay same angle";
        assertTrue(new HerdAngle(-10).getAngle() == -10); // "Negative in valid range failed to stay same angle";

        // Rotate positive - No wrap
        assertTrue(new HerdAngle(10).rotate(30).getAngle() == 40); // "Positive in failed to rotate positive direction";
        assertTrue(new HerdAngle(-10).rotate(30).getAngle() == 20); // "Negative in failed to rotate positive direction";

        // Rotate negative - No wrap
        assertTrue(new HerdAngle(10).rotate(-30).getAngle() == -20); // "Positive in failed to rotate negative direction";
        assertTrue(new HerdAngle(-10).rotate(-30).getAngle() == -40); // "Negative in failed to rotate negative direction";
    }

    @Test
    public void testWrap()
    {
        // Constructor - Wrap
        assertTrue(new HerdAngle(190).getAngle() == -170); // "Positive outside valid range failed to wrap angle";
        assertTrue(new HerdAngle(-190).getAngle() == 170); // "Negative outside valid range failed to wrap angle";

        // Rotate positive - Wrap
        assertTrue(new HerdAngle(10).rotate(190).getAngle() == -160); // "Positive in failed to rotate positive and wrap negative";
        assertTrue(new HerdAngle(-10).rotate(210).getAngle() == -160); // "Negative in failed to rotate positive and wrap negative";

        // Rotate negative - Wrap
        assertTrue(new HerdAngle(10).rotate(-210).getAngle() == 160); // "Positive in failed to rotate negative and wrap positive";
        assertTrue(new HerdAngle(-10).rotate(-190).getAngle() == 160); // "Negative in failed to rotate negative and wrap positive";

        // Rotate WrappedAngle
        assertTrue(new HerdAngle(10).rotate(new HerdVector(1, 30)).getAngle() == 40); // "HerdAngle cannot rotate by positive HerdVector";
        assertTrue(new HerdAngle(10).rotate(new HerdVector(1, -30)).getAngle() == -20); // "HerdAngle cannot rotate by negative HerdVector";
        assertTrue(new HerdAngle(10).rotateReverse(new HerdVector(1, 30)).getAngle() == -20); // "HerdAngle cannot inverse rotate by positive HerdVector";
        assertTrue(new HerdAngle(10).rotateReverse(new HerdVector(1, -30)).getAngle() == 40); // "HerdAngle cannot inverse rotate by negative HerdVector";
    }
}
