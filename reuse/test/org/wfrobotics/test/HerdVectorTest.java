package org.wfrobotics.test;

import org.junit.Test;
import org.wfrobotics.reuse.math.HerdAngle;
import org.wfrobotics.reuse.math.HerdVector;

// TODO cross broken? parallel should have mag zero

/** To enable unit tests: Tests.java->Properties->Run/debug settings->Tests->Edit->Arguments->VM args->-ea */
public class HerdVectorTest
{
    @Test
    public void debugHerdVector()
    {
        // New Vector: Trig
        HerdVector a;
        HerdVector b;

        a = new HerdVector(1, 60);

        // Constructor: Positive mag
        assert new HerdVector(-3, 10).getMag() == 3 : "Positive mag on > 1 mag";
        assert new HerdVector(-1, 10).getMag() == 1 : "Positive mag on 1 mag";
        assert new HerdVector(-.125, 10).getMag() == .125 : "Positive < 1 mag";

        // New Vector: Zero
        b = new HerdVector(0, 180);
        System.out.println("(0, 0): " + b);
        System.out.println();

        // Scale - By positive
        assert new HerdVector(2, 30).scale(3).getMag() == 6 : "Scale by bigger positive double failed mag";
        assert new HerdVector(2, 30).scale(3).getAngle() == 30 : "Scale by bigger positive double failed angle same";
        assert new HerdVector(2, 30).scale(1).getMag() == 2 : "Scale by double 1 failed mag same";
        assert new HerdVector(2, 30).scale(1).getAngle() == 30 : "Scale by double 1 failed angle same";
        assert new HerdVector(2, 30).scale(.125).getMag() == .25 : "Scale by smaller positive double failed mag";
        assert new HerdVector(2, 30).scale(.125).getAngle() == 30 : "Scale by smaller positive double failed angle same";

        // Scale - By negative
        assert new HerdVector(2, 30).scale(-3).getMag() == 6 : "Scale by positive double failed mag";
        assert new HerdVector(2, 30).scale(-3).getAngle() == -150 : "Scale by positive double failed angle flip";
        assert new HerdVector(2, 30).scale(-1).getMag() == 2 : "Scale by double -1 failed mag";
        assert new HerdVector(2, 30).scale(-1).getAngle() == -150 : "Scale by double -1 failed angle flip";
        assert new HerdVector(2, 30).scale(-.125).getMag() == .25 : "Scale by negative double failed mag";
        assert new HerdVector(2, 30).scale(-.125).getAngle() == -150 : "Scale by negative double failed angle flip";

        // Clone
        a = new HerdVector(1, 10);
        b = new HerdVector(a);
        b = b.rotate(45);
        assert a.getAngle() == 10 : "Clone overwrites original";

        // Addition
        a = new HerdVector(1, 0);
        b = new HerdVector(1, 90);

        System.out.println(a.add(b));
        System.out.println();

        assert new HerdVector(1, 135).add(new HerdVector(1, 135)).getMag() == 2 : "Add HerdVector to self not twice mag";
        assert new HerdVector(1, 135).add(new HerdVector(1, 135)).getAngle() != 135 : "Add HerdVector to self not same angle";

        // Cross
        a = new HerdVector(2, 0);
        b = new HerdVector(3, 85);

        System.out.format(a + " cross " + b + " = " + a.cross(b) + "\n");
        b = b.rotate(180);
        System.out.format(a + " cross " + b + " = " + a.cross(b) + "\n");

        // Cross - Limits
        //assert new HerdVector(2, 0).cross(new HerdVector(3, 0)).getMag() == 0 : "Cross zero mag on parallel";
        assert new HerdVector(2, 0).cross(new HerdVector(3, 90)).getMag() == 6 : "Cross max mag on orthogonal";

        // Clamp
        assert new HerdVector(.44, 10).clampToRange(.45, .55).getMag() == .45 : "Clamp min";
        assert new HerdVector(.5, 10).clampToRange(.45, .55).getMag() == .5 : "Clamp unaffected";
        assert new HerdVector(.56, 10).clampToRange(.45, .55).getMag() == .55 : "Clamp max";
    }

    @Test
    public void debugHerdVectorWrappedAngle()
    {
        // Constructor - No wrap
        assert new HerdVector(1, 10).getAngle() == 10 : "Positive in valid range failed to stay same angle";
        assert new HerdVector(1, -10).getAngle() == -10 : "Negative in valid range failed to stay same angle";

        // Rotate positive - No wrap
        assert new HerdVector(1, 10).rotate(30).getAngle() == 40 : "Positive in failed to rotate positive direction";
        assert new HerdVector(1, -10).rotate(30).getAngle() == 20 : "Negative in failed to rotate positive direction";

        // Rotate negative - No wrap
        assert new HerdVector(1, 10).rotate(-30).getAngle() == -20 : "Positive in failed to rotate negative direction";
        assert new HerdVector(1, -10).rotate(-30).getAngle() == -40 : "Negative in failed to rotate negative direction";

        // Constructor - Wrap
        assert new HerdVector(1, 190).getAngle() == -170 : "Positive outside valid range failed to wrap angle";
        assert new HerdVector(1, -190).getAngle() == 170 : "Negative outside valid range failed to wrap angle";

        // Rotate positive - Wrap
        assert new HerdVector(1, 10).rotate(190).getAngle() == -160 : "Positive in failed to rotate positive and wrap negative";
        assert new HerdVector(1, -10).rotate(210).getAngle() == -160 : "Negative in failed to rotate positive and wrap negative";

        // Rotate negative - Wrap
        assert new HerdVector(1, 10).rotate(-210).getAngle() == 160 : "Positive in failed to rotate negative and wrap positive";
        assert new HerdVector(1, -10).rotate(-190).getAngle() == 160 : "Negative in failed to rotate negative and wrap positive";

        // Rotate WrappedAngle
        assert new HerdVector(1, 30).rotate(new HerdAngle(10)).getAngle() == 40 : "HerdVector cannot rotate by positive HerdAngle";
        assert new HerdVector(1, 30).rotate(new HerdAngle(-10)).getAngle() == 20 : "HerdVector cannot rotate by negative HerdAngle";
        assert new HerdVector(1, 30).rotateReverse(new HerdAngle(10)).getAngle() == 20 : "HerdVector cannot inverse rotate by positive HerdAngle";
        assert new HerdVector(1, 30).rotateReverse(new HerdAngle(-10)).getAngle() == 40 : "HerdVector cannot inverse rotate by negative HerdAngle";
    }
}
