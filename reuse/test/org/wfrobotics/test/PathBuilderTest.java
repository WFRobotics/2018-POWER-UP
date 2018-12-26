package org.wfrobotics.test;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;
import org.wfrobotics.reuse.config.PathBuilder;
import org.wfrobotics.reuse.config.PathBuilder.Waypoint;
import org.wfrobotics.reuse.math.geometry.Translation2d;
import org.wfrobotics.reuse.math.motion.MotionState;
import org.wfrobotics.reuse.subsystems.control.Path;
import org.wfrobotics.reuse.subsystems.control.PathSegment;
import org.wfrobotics.reuse.subsystems.drive.TankMaths;

public class PathBuilderTest
{
    static double velocity = TankMaths.kVelocityMaxInchesPerSecond;

    public static Path getDriveDistance()
    {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 17 * 12;
        double y1 = 0;

        double endSpeed = 0.0;
        MotionState stopped = new MotionState(0, 0, 0, 0);

        Path path = new Path();
        path.addSegment(new PathSegment(x0, y0, x1, y1, velocity, stopped, endSpeed));
        return path;
    }

    public static Path getPathFromWaypoints()
    {
        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new Waypoint(0,  0,  0, 0));
        waypoints.add(new Waypoint(17 * 12,  0, 0, velocity));
        return PathBuilder.buildPathFromWaypoints(waypoints);
    }

    @Test
    public void testBuildsDriveDistance()
    {
        double e = 0.1;
        ArrayList<Double> distances = new ArrayList<Double>();
        Path path = getPathFromWaypoints();
        Path path2 = getDriveDistance();

        distances.add(path2.getEndPosition().x() * -0.1);
        distances.add(path2.getEndPosition().x() * 0.0);
        distances.add(path2.getEndPosition().x() * 0.1);
        distances.add(path2.getEndPosition().x() * 0.5);
        distances.add(path2.getEndPosition().x() * 0.9);
        distances.add(path2.getEndPosition().x() * 1.0);

        System.out.println(path);
        System.out.println(path2);

        path.verifySpeeds();
        path2.verifySpeeds();

        assertEquals(path.getSegmentLength(), path2.getSegmentLength(), e);
        assertEquals(path.getSegmentRemainingDist(new Translation2d(0, 0)), path2.getSegmentRemainingDist(new Translation2d(0, 0)), e);
        assertEquals(path.getEndPosition().x(), path2.getEndPosition().x(), e);
        assertEquals(path.getEndPosition().y(), path2.getEndPosition().y(), e);
        assertEquals(path.getEndPosition().direction().getDegrees(), path2.getEndPosition().direction().getDegrees(), e);
        assertEquals(path.getLastMotionState().t(), path2.getLastMotionState().t(), e);
        assertEquals(path.getLastMotionState().pos(), path2.getLastMotionState().pos(), e);
        assertEquals(path.getLastMotionState().vel(), path2.getLastMotionState().vel(), e);
        assertEquals(path.getLastMotionState().acc(), path2.getLastMotionState().acc(), e);
        for(Double dist : distances)
        {
            Translation2d tOnPath = new Translation2d(dist, 0);
            Translation2d tOffset = new Translation2d(dist, 1);

            assertEquals(path.getSpeed(tOnPath), path2.getSpeed(tOnPath), e);
            assertEquals(path.getSpeed(tOffset), path2.getSpeed(tOffset), e);
            assertEquals(path.getSegmentRemainingDist(tOffset), path.getSegmentRemainingDist(tOffset), e);
            path.checkSegmentDone(tOffset);
            path2.checkSegmentDone(tOffset);
        }
    }
}
