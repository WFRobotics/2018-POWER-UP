package org.wfrobotics.test.vision;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

public class Extrapolate
{
    @Test
    public void simpleTest()
    {
        // simple
        assertEquals(extrapolate(1, 1.5), 1.5, 0.001);
        // negative
        assertEquals(extrapolate(-1, 1.5), -1.5, 0.001);
        // zero
        assertEquals(extrapolate(-1, 0), 0, 0.001);
    }

    @Test
    public void lessSimpleTest()
    {
        assertEquals(extrapolate(0.0, 1, 1.5), 1.5,  0.001);

        assertEquals(extrapolate(0.5, 1, 1.5), 1.25, 0.001);

        assertEquals(extrapolate(1.0, 2, 10), 11, 0.001);

        assertEquals(extrapolate(1.0, 2, 10), 11, 0.001);


    }
    @Test
    public void pointsTest()
    {
        List<Points> list = new ArrayList<Points>();
        list.add(new Points(0, 0));
        list.add(new Points(1, 1.5));

        assertEquals(extropolate(list, 1.5), 1.5, 0.001 );
        list.clear();

        list.add(new Points(0.5, 0.5));
        list.add(new Points(1, 1));
        assertEquals(extropolate(list, 1.5), 1.25, 0.001 );

    }
    public double extropolate(List<Points> points, double percentExtrapolation)
    {
        Points start = points.get(0);
        Points valueNow = points.get(1);
        //        return (valueNow - start) * percentExtrapolation + start;
        return (valueNow.time - start.time) * percentExtrapolation + start.time;
    }
    public double extrapolate(double start, double valueNow, double percentExtrapolation)
    {
        return (valueNow - start) * percentExtrapolation + start;
    }

    public double extrapolate(double input, double percentExtrapolation)
    {
        return input * percentExtrapolation;
    }

}
class Points
{
    final double time;
    final double error;
    public Points(double time, double err)
    {
        error = err;
        this.time = time;
    }
}
