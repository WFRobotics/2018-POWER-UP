package org.wfrobotics.test;

import org.junit.Test;
import org.wfrobotics.reuse.math.Util;

public class UtilitiesTest
{
    @Test
    public void testUtilities()
    {
        assert Util.clampToRange(-0.75, -1.25, 1.25) == -0.75;
        assert Util.clampToRange(0.75, -1.25, 1.25) == 0.75;
        assert Util.clampToRange(-1.25, -1.25, 1.25) == -1.25;
        assert Util.clampToRange(1.25, -1.25, 1.25) == 1.25;
        assert Util.clampToRange(-2.5, -1.25, 1.25) == -1.25;
        assert Util.clampToRange(2.5, -1.25, 1.25) == 1.25;

        assert Util.clampToRange(2.5, 1.25) == 1.25;
        assert Util.clampToRange(-2.5, 1.25) == -1.25;
        assert Util.clampToRange(2.5, 1.25) == 1.25;
        assert Util.clampToRange(0.0, 1.25) == 0.0;

        assert Util.scaleToRange(0.0, 0.0, 1.25) == 0.0;
        assert Util.scaleToRange(0.625, 1.25, 2.5) == 1.25;
        assert Util.scaleToRange(1.25, 1.25, 1.25) == 1.25;

        assert Util.scaleToRange(1.25, 50.0, 100.0, 1.0, 2.0) == 1.0;
        assert Util.scaleToRange(125.0, 50.0, 100.0, 1.0, 2.0) == 2.0;
        assert Util.scaleToRange(1.25, 0.0, 1.25, 1.25, 2.5) == 2.5;
        assert Util.scaleToRange(0.0, 0.0, 1.25, 1.25, 2.5) == 1.25;
        assert Util.scaleToRange(0.625, 0.0, 1.25, 1.25, 2.5) == 1.875;
        assert Util.scaleToRange(-1.25, -1.25, 1.25, 1.25, 2.5) == 1.25;
        assert Util.scaleToRange(1.25, -1.25, 1.25, 1.25, 2.5) == 2.5;
    }
}
