package org.wfrobotics.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.Test;
import org.wfrobotics.reuse.config.PathBuilder;
import org.wfrobotics.reuse.config.PathBuilder.Waypoint;
import org.wfrobotics.reuse.math.geometry.ITranslation2d;
import org.wfrobotics.reuse.math.geometry.Pose2d;
import org.wfrobotics.reuse.math.geometry.Rotation2d;
import org.wfrobotics.reuse.math.geometry.Translation2d;
import org.wfrobotics.reuse.math.motion.MotionState;
import org.wfrobotics.reuse.subsystems.control.Path;
import org.wfrobotics.reuse.subsystems.control.PathSegment;

public class PathAdaptorTest
{
    static final double kEpsilon = 1E-9;

    @Test
    public void testAdaptorSimpleLine()
    {
        List<Pose2d> poses = Arrays.asList(new Pose2d[] {
            new Pose2d(50, 50, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 50, Rotation2d.fromDegrees(0)),
        });
        List<Waypoint> waypoints = Arrays.asList(new Waypoint[] {
            new Waypoint(50,50,0,0),
            new Waypoint(100,50,0,60),
        });
        Path p1 = PathBuilder.buildPathFromWaypoints(waypoints);
        Path p2 = buildPathFromPoses(poses, waypoints.get(1).speed, waypoints.get(waypoints.size() - 1).speed);
        checkAdaptorEquivalence(p1, p2);
    }

    public static List<Waypoint> getWaypointCorner()
    {
        return Arrays.asList(new Waypoint[] {
            new Waypoint(50,50,0,0),
            new Waypoint(100,50,0,120),
            new Waypoint(100,100,0,60),
        });
    }

    public static List<Pose2d> getPoseCorner()
    {
        return Arrays.asList(new Pose2d[] {
            new Pose2d(50, 50, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 50, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 100, Rotation2d.fromDegrees(90)),
        });
    }

    @Test
    public void testAdaptorQuarterCircle()
    {
        List<Pose2d> poses = Arrays.asList(new Pose2d[] {
            new Pose2d(50, 50, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 100, Rotation2d.fromDegrees(90)),
        });
        List<Waypoint> waypoints = Arrays.asList(new Waypoint[] {
            new Waypoint(50,50,0,0),
            new Waypoint(100,50,50,120),
            new Waypoint(100,100,0,60),
        });
        Path p1 = PathBuilder.buildPathFromWaypoints(waypoints);
        Path p2 = buildPathFromPoses(poses, waypoints.get(1).speed, waypoints.get(waypoints.size() - 1).speed);
        checkAdaptorEquivalence(p1, p2);
    }

    public static List<Waypoint> getWaypointRoundedBox()
    {
        return Arrays.asList(new Waypoint[] {
            new Waypoint(50,50,0,0),
            new Waypoint(100,50,25,120),
            new Waypoint(100,100,0,60),
        });
    }

    @Test
    public void testAdaptorSCurve()
    {
        List<Pose2d> poses = Arrays.asList(new Pose2d[] {
            new Pose2d(50, 50, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 100, Rotation2d.fromDegrees(90)),
            new Pose2d(150, 150, Rotation2d.fromDegrees(0)),
        });
        List<Waypoint> waypoints = Arrays.asList(new Waypoint[] {
            new Waypoint(50,50,0,0),
            new Waypoint(100,50,50,120),
            new Waypoint(100,100,0,120),
            new Waypoint(100,150,50,120),
            new Waypoint(150,150,0,60),
        });
        Path p1 = PathBuilder.buildPathFromWaypoints(waypoints);
        Path p2 = buildPathFromPoses(poses, waypoints.get(1).speed, waypoints.get(waypoints.size() - 1).speed);
        checkAdaptorEquivalence(p1, p2);
    }

    public static List<Waypoint> getWaypointOval()
    {
        return Arrays.asList(new Waypoint[] {
            new Waypoint(50,200,0,0),
            //        new Waypoint(100,200,0,120);
            new Waypoint(100,200,0,120),
            new Waypoint(300,200,50,120),
            new Waypoint(300,100,50,120), // TODO Messed with this
            new Waypoint(100,100,50,120),
            new Waypoint(100,200,50,120),
            new Waypoint(150,200,0,60),
        });
    }

    public static List<Pose2d> getPoseOval()
    {
        return Arrays.asList(new Pose2d[] {
            new Pose2d(50, 200, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 200, Rotation2d.fromDegrees(0)),
            new Pose2d(250, 200, Rotation2d.fromDegrees(0)),
            new Pose2d(300, 150, Rotation2d.fromDegrees(-90)),
            new Pose2d(150, 100, Rotation2d.fromDegrees(-180)),
            new Pose2d(100, 150, Rotation2d.fromDegrees(90)),
            new Pose2d(150, 200, Rotation2d.fromDegrees(0)),
        });
    }

    //    @Test
    //    public void testAdaptorOval()
    //    {
    //        List<Pose2d> poses = getPoseOval();
    //        List<Waypoint> waypoints = getWaypointOval();
    //        Path p1 = PathBuilder.buildPathFromWaypoints(waypoints);
    //        Path p2 = buildPathFromPoses(poses, waypoints.get(1).speed, waypoints.get(waypoints.size() - 1).speed);
    //        checkAdaptorEquivalence(p1, p2);
    //    }

    @Test
    public void testArcsSimpleLine()
    {
        List<Pose2d> poses = Arrays.asList(new Pose2d[] {
            new Pose2d(50, 50, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 50, Rotation2d.fromDegrees(0)),
        });
        List<Waypoint> waypoints = Arrays.asList(new Waypoint[] {
            new Waypoint(50,50,0,0),
            new Waypoint(100,50,0,60),
        });
        double maxSpeed = 120.0;
        double endSpeed = 60.0;
        Path p1 = PathBuilder.buildPathFromWaypoints(waypoints);
        Path p2 = buildArcPoses(poses, maxSpeed, endSpeed);
        checkAdaptorEquivalence(p1, p2);
    }

    @Test
    public void testArcsQuarterCircle()
    {
        List<Pose2d> poses = Arrays.asList(new Pose2d[] {
            new Pose2d(50, 50, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 100, Rotation2d.fromDegrees(90)),
        });
        List<Waypoint> waypoints = Arrays.asList(new Waypoint[] {
            new Waypoint(50,50,0,0),
            new Waypoint(100,50,50,120),
            new Waypoint(100,100,0,60),
        });
        double maxSpeed = 120.0;
        double endSpeed = 60.0;
        Path p1 = PathBuilder.buildPathFromWaypoints(waypoints);
        Path p2 = buildArcPoses(poses, maxSpeed, endSpeed);
        checkAdaptorEquivalence(p1, p2);
    }

    @Test
    public void testArcsSCurve()
    {
        List<Pose2d> poses = Arrays.asList(new Pose2d[] {
            new Pose2d(50, 50, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 100, Rotation2d.fromDegrees(90)),
            new Pose2d(150, 150, Rotation2d.fromDegrees(0)),
        });
        List<Waypoint> waypoints = Arrays.asList(new Waypoint[] {
            new Waypoint(50,50,0,0),
            new Waypoint(100,50,50,120),
            new Waypoint(100,100,0,120),
            new Waypoint(100,150,50,120),
            new Waypoint(150,150,0,60),
        });
        double maxSpeed = 120.0;
        double endSpeed = 60.0;
        Path p1 = PathBuilder.buildPathFromWaypoints(waypoints);
        Path p2 = buildArcPoses(poses, maxSpeed, endSpeed);
        checkAdaptorEquivalence(p1, p2);
    }

    @Test
    public void testArcsOval()
    {
        List<Pose2d> poses = Arrays.asList(new Pose2d[] {
            new Pose2d(50, 200, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 200, Rotation2d.fromDegrees(0)),
            new Pose2d(250, 200, Rotation2d.fromDegrees(0)),
            new Pose2d(300, 150, Rotation2d.fromDegrees(-90)),
            new Pose2d(150, 100, Rotation2d.fromDegrees(-180)),
            new Pose2d(100, 150, Rotation2d.fromDegrees(90)),
            new Pose2d(150, 200, Rotation2d.fromDegrees(0)),
        });
        List<Waypoint> waypoints = Arrays.asList(new Waypoint[] {
            new Waypoint(50,200,0,0),
            //        new Waypoint(100,200,0,120);
            new Waypoint(100,200,0,120),
            new Waypoint(300,200,50,120),
            new Waypoint(300,100,50,120), // TODO Messed with this
            new Waypoint(100,100,50,120),
            new Waypoint(100,200,50,120),
            new Waypoint(150,200,0,60),
        });
        double maxSpeed = 120.0;
        double endSpeed = 60.0;
        Path p1 = PathBuilder.buildPathFromWaypoints(waypoints);
        Path p2 = buildArcPoses(poses, maxSpeed, endSpeed);
        checkAdaptorEquivalence(p1, p2);
    }

    public static Path buildArcPoses(List<Pose2d> poses, double maxSpeed, double endSpeed)
    {
        Path p = new Path();

        for (int index = 0; index < poses.size() - 1; index++)
        {
            Pose2d current = poses.get(index);
            Pose2d next = poses.get(index + 1);
            Translation2d start = current.getTranslation();
            Translation2d end = next.getTranslation();
            boolean isLast = index == poses.size() - 2;
            double velocity;
            double velocityEnd = (isLast) ? maxSpeed : 0.0;
            PathSegment s;

            if (current.isColinear(next))
            {
                velocity = (isLast) ? endSpeed : maxSpeed;
                s = new PathSegment(start.x(), start.y(), end.x(), end.y(), velocity, p.getLastMotionState(), velocityEnd);
            }
            else
            {
                Arc<Pose2d> arc = new Arc<Pose2d>(current, next);

                // TODO This condition means we need to add a line to be smooth
                System.out.println(String.format("%.0f, %.0f", start.x() - end.x(), start.y() - end.y()));
                double smallestSide = Math.min(Math.abs(start.x() - end.x()), Math.abs(start.y() - end.y()));
                if (Math.abs(arc.radius) > smallestSide)
                {
                    System.out.println("oh snap");
                }

                velocity = (isLast) ? (maxSpeed + endSpeed) / 2.0 : maxSpeed;
                System.out.println(arc);
                s = new PathSegment(start.x(), start.y(), end.x(), end.y(), arc.center.x(), arc.center.y(), velocity, p.getLastMotionState(), velocityEnd);
            }
            p.addSegment(s);
        }
        p.extrapolateLast();
        p.verifySpeeds();
        return p;
    }

    public static class Arc<S extends ITranslation2d<S>> {
        public Translation2d center;
        public double radius;
        public double length;

        public Arc(final Pose2d pose, final S point) {
            center = findCenter(pose, point);
            radius = new Translation2d(center, point.getTranslation()).norm();
            length = findLength(pose, point, center, radius);
            radius *= getDirection(pose, point);
        }

        protected Translation2d findCenter(Pose2d pose, S point) {
            final Translation2d poseToPointHalfway = pose.getTranslation().interpolate(point.getTranslation(), 0.5);
            final Rotation2d normal = pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction()
                                            .normal();
            final Pose2d perpendicularBisector = new Pose2d(poseToPointHalfway, normal);
            final Pose2d normalFromPose = new Pose2d(pose.getTranslation(),
                                            pose.getRotation().normal());
            if (normalFromPose.isColinear(perpendicularBisector.normal())) {
                // Special case: center is poseToPointHalfway.
                return poseToPointHalfway;
            }
            return normalFromPose.intersection(perpendicularBisector);
        }

        protected double findLength(Pose2d pose, S point, Translation2d center, double radius) {
            if (radius < Double.MAX_VALUE) {
                final Translation2d centerToPoint = new Translation2d(center, point.getTranslation());
                final Translation2d centerToPose = new Translation2d(center, pose.getTranslation());
                // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
                // check the sign of the cross-product between the normal vector and the vector from pose to point.
                final boolean behind = Math.signum(
                                                Translation2d.cross(pose.getRotation().normal().toTranslation(),
                                                                                new Translation2d(pose.getTranslation(), point.getTranslation()))) > 0.0;
                                                                                final Rotation2d angle = Translation2d.getAngle(centerToPose, centerToPoint);
                                                                                return radius * (behind ? 2.0 * Math.PI - Math.abs(angle.getRadians()) : Math.abs(angle.getRadians()));
            }
            return new Translation2d(pose.getTranslation(), point.getTranslation()).norm();
        }

        protected static <S extends ITranslation2d<S>> double getDirection(Pose2d pose, S point) {
            Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point.getTranslation());
            Translation2d robot = pose.getRotation().toTranslation();
            double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
            return (cross < 0.) ? -1. : 1.; // if robot < pose turn left
        }

        public String toString()
        {
            return String.format("C: %s, R: %.0f, L: %.0f", center, radius, length);
        }
    }

    public void checkAdaptorEquivalence(Path p1, Path p2)
    {
        System.out.println("---\nPath Dump:");
        System.out.println(p1);
        System.out.println(p2);

        Translation2d end0 = p1.getEndPosition();
        MotionState last0 = p1.getLastMotionState();
        MotionState last1 = p2.getLastMotionState();

        assertTrue(p1.getSegmentLength() == p2.getSegmentLength());
        assertEquals(p1.getSegmentRemainingDist(end0), p2.getSegmentRemainingDist(end0), kEpsilon);
        assertTrue(p1.getEndPosition().equals(p2.getEndPosition()));
        assertTrue(last0.equals(last1));
        assertEquals(last0.vel(), last1.vel(), kEpsilon);
        System.out.println("---");
    }

    public static Path buildPathFromPoses(List<Pose2d> poses, double maxSpeed, double endSpeed)
    {
        Path path = new Path();

        for (int index = 0; index < poses.size() - 1; index++)
        {
            Pose2d current = poses.get(index);
            Pose2d next = poses.get(index + 1);
            double speed = (index == poses.size() - 2) ? (maxSpeed + endSpeed) / 2.0 : maxSpeed;
            double end = (index == poses.size() - 2) ? maxSpeed : 0.0;

            addLineSegment(path, current, next, speed, end);
        }
        path.extrapolateLast();
        path.verifySpeeds();

        return path;
    }

    public static void addLineSegment(Path path, Pose2d start, Pose2d end, double maxSpeed, double endSpeed)
    {
        double x1 = start.getTranslation().x();
        double y1 = start.getTranslation().y();
        double x2 = end.getTranslation().x();
        double y2 = end.getTranslation().y();
        double speed = maxSpeed;
        MotionState endState = path.getLastMotionState();
        PathSegment segment;

        if (start.isColinear(end))
        {
            segment = new PathSegment(x1, y1, x2, y2, speed, endState, endSpeed);
        }
        else
        {
            Translation2d center = intersect(start, end);
            //            double radius = new Translation2d(center, end.getTranslation()).norm();

            segment = new PathSegment(x1, y1, x2, y2, center.x(), center.y(), speed, endState, endSpeed);
        }
        path.addSegment(segment);
    }

    public static Translation2d intersect(Pose2d one, Pose2d two)
    {
        final Pose2d lineA = one.normal();
        final Pose2d lineB = two.normal();
        return lineA.intersection(lineB);
    }

    //    @Test
    //    public void testWaypointPoseAdaptor()
    //    {
    //        List<Waypoint> waypoints = getWaypointOval();
    //        List<Pose2d> posepoints = getPoseOval();
    //        List<Waypoint> builtpoints = PathBuilder.adaptWaypointsFromPoses(posepoints, waypoints.get(1).speed, waypoints.get(waypoints.size() - 1).speed);
    //
    //        System.out.println();
    //        for (int index = 0; index < waypoints.size(); index++)
    //        {
    //            Waypoint w = waypoints.get(index);
    //            Waypoint n = builtpoints.get(index);
    //            System.out.println(String.format("%d| %s", index, w));
    //            System.out.println(String.format("%d| %s", index, n));
    //            assertTrue(w.position.equals(n.position));
    //            assertEquals(w.radius, n.radius, kEpsilon);
    //            assertEquals(w.speed, n.speed, kEpsilon);
    //        }
    //        assertTrue(waypoints.size() > 2);
    //        assertTrue(waypoints.size() == builtpoints.size());
    //    }

    //    @Test
    //    public void testWaypointPathFromPoses()
    //    {
    //        List<Waypoint> waypoints = getWaypointOval();
    //        List<Pose2d> posepoints = getPoseOval();
    //
    //        Path path0 = PathBuilder.buildPathFromWaypoints(waypoints);
    //        Path path1 = PathBuilder.buildPathFromPoses(posepoints, waypoints.get(1).speed, waypoints.get(waypoints.size() - 1).speed);
    //        Translation2d last = posepoints.get(posepoints.size() - 1).getTranslation();
    //
    //        assertTrue(waypoints.size() > 2);
    //        assertTrue(waypoints.size() == posepoints.size());
    //
    //        assertTrue(path0.getSegmentLength() == path1.getSegmentLength()));
    //        assertEquals(path0.getSegmentRemainingDist(last), path1.getSegmentRemainingDist(last), kEpsilon);
    //        assertTrue(path0.getEndPosition().equals(path1.getEndPosition()));
    //
    //        System.out.println(path0);
    //        System.out.println();
    //        System.out.println(path1);
    //    }

    //    public static List<Waypoint> adaptWaypointsFromPoses(List<Pose2d> posepoints, double maxVelocity, double endVelocity)
    //    {
    //        final double kEpsilon = 1E-9;
    //        final double kReallyBigNumber = 1E9;
    //        final List<Waypoint> newpoints = new ArrayList<Waypoint>();
    //
    //        // First waypoint is where the robot starts, always the same as the pose
    //        Pose2d first = posepoints.get(0);
    //        newpoints.add(new Waypoint(first.getTranslation(), first.getRotation().getDegrees(), 0.0));
    //
    //        // Additional waypoints may curve, defined by an arc on the edge of some circle
    //        for (int index = 1; index < posepoints.size() - 1; index++)
    //        {
    //            // Robot at the beginning and end of this motion
    //            Pose2d start = posepoints.get(index);
    //            Pose2d end = posepoints.get(index + 1);
    //
    //            // If the poses have the same heading, the waypoint is just a line between the poses
    //            if(start.isColinear(end))
    //            {
    //                newpoints.add(new Waypoint(start.getTranslation(), start.getRotation().getDegrees(), maxVelocity));
    //                continue;
    //            }
    //
    //            // Otherwise find a circle that arcs through both poses and is parallel to each's heading
    //            Translation2d center = intersect(start, end);
    //            double radius = new Translation2d(center, end.getTranslation()).norm();
    //
    //            if (radius > kEpsilon && radius < kReallyBigNumber)
    //            {
    //                ConsoleLogger.warning("buildPathFromPoses() math may have exploded");
    //            }
    //
    //            // So which part of that circle is the waypoint? None of it. Actually
    //            // the two poses, the circle's center, and the waypoint form a rectangle.
    //
    //            // Find a line between the poses
    //            Translation2d startpoint = start.getTranslation();
    //            Translation2d endpoint = end.getTranslation();
    //
    //            // Find the middle of the line between the poses, we'll mirror across here
    //            Translation2d lineMiddle = startpoint.interpolate(endpoint, .5);
    //
    //            // Our line's middle to the circle's center is how far we need to flip the circle's center point
    //            Translation2d centerToMiddle = lineMiddle.translateBy(center.inverse());
    //
    //            // Mirror the circle's center across our line
    //            Translation2d waypoint = lineMiddle.translateBy(centerToMiddle);
    //            newpoints.add(new Waypoint(waypoint, radius, maxVelocity));
    //        }
    //
    //        // Last waypoint is where the robot ends, always the same as the pose
    //        Pose2d last = posepoints.get(posepoints.size() - 1);
    //        newpoints.add(new Waypoint(last.getTranslation(), last.getRotation().getDegrees(), endVelocity));
    //
    //        return newpoints;
    //    }
}
