package org.wfrobotics.test.vision;

import java.util.ArrayList;
import java.util.List;

import org.wfrobotics.reuse.RobotStateBase;
import org.wfrobotics.reuse.subsystems.vision.CoprocessorData;
import org.wfrobotics.reuse.subsystems.vision.CoprocessorData.VisionTargetInfo;

public class RobotState extends RobotStateBase
{
    public void TODO()
    {
        // --- RIO ---
        // DONE Socket connect/disconnect pretty robust now
        // DONE RIO code up to RobotState pretty cleaned up now
        // TODO Need to integrate cleaned up classes with actual robot code.

        // Interpolation & Timestamps
        // Now thinking we want to compute latency rather than try syncing our time through NTP
        // DONE Copy 254 vision to here, already ported to bottom of real robot's robot state
        // DONE Try feeding our info into that code, can we get an interpolated value out that makes sense?
        // DONE Started to do timestamping (capture time now recorded). Adjust for capture to use latency.

        // Integration with Robot
        // TODO Understand Goal Tracker and Goal Track output, what are the units, when does it change, etc
        // TODO Create vision Command using getAimingParameters() somewhat like 254's 2017 Drive subsystem

        // --- Kangeroo ---
        // DONE Maybe can do timestamping better for latency computation. Low priority.
        // DONE May want to go down to two processes? Low priority.
        // DRL I think the coprocessor is good enough for now
    }

    private static final RobotState instance = new RobotState();

    public CoprocessorData update;
    public  List<Point> points = new ArrayList<Point>();

    public static RobotState getInstance()
    {
        return instance;
    }

    public void addVisionUpdate(Double time, CoprocessorData coprocessorData)
    {
        update = coprocessorData;

        if (coprocessorData.targets.size() > 0)
        {
            visionInView = true;

            VisionTargetInfo largestTarget = update.targets.get(0);
            for (VisionTargetInfo target : update.targets)
            {
                if ( target.area() > largestTarget.area() || largestTarget == null)
                {
                    largestTarget = target;
                }
            }
            points.add(0, (new Point(time, largestTarget)));
        }
        else {
            visionInView = false;
        }
        if (points.size() > 3)
        {
            printFakeState();
        }
    }
    public void printFakeState()
    {
        System.out.println(getVisionError());
    }
    public double getVisionError()
    {
        return points.get(0).extrapolate(points.get(0), points.get(0).time).x_error;
    }

    public double extropolate(List<Point> points, double percentExtrapolation)
    {
        Point start = points.get(1);
        Point valueNow = points.get(0);
        return (valueNow.time - start.time) * percentExtrapolation + start.time;
    }
    protected void resetRobotSpecificState()
    {
    }

    public void addVisionUpdate(CoprocessorData latest)
    {
    }

}
//        List<Translation2d> field_to_goals = new ArrayList<>();
//        Pose2d field_to_camera = getFieldToCamera(timestamp);
//        if (coprocessorData.isValid() && coprocessorData.targets.size() > 0)
//        {
//            for (VisionTargetInfo target : coprocessorData.targets)
//            {
//                double ydeadband = (target.getY() > -kCameraDeadband && target.getY() < kCameraDeadband) ? 0.0 : target.getY();
//
//                // Compensate for camera yaw
//                double xyaw = target.getX() * camera_yaw_correction_.cos() + ydeadband * camera_yaw_correction_.sin();
//                double yyaw = ydeadband * camera_yaw_correction_.cos() - target.getX() * camera_yaw_correction_.sin();
//                double zyaw = target.getZ();
//
//                // Compensate for camera pitch
//                double xr = zyaw * camera_pitch_correction_.sin() + xyaw * camera_pitch_correction_.cos();
//                double yr = yyaw;
//                double zr = zyaw * camera_pitch_correction_.cos() - xyaw * camera_pitch_correction_.sin();
//
//                // find intersection with the goal
//                if (zr > 0)
//                {
//                    double scaling = differential_height_ / zr;
//                    double distance = Math.hypot(xr, yr) * scaling + kBoilerRadius;
//                    Rotation2d angle = new Rotation2d(xr, yr, true);
//                    field_to_goals.add(field_to_camera.transformBy(Pose2d.fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin()))).getTranslation());
//                }
//            }
//        }
//        synchronized (this)
//        {
//            goal_tracker_.update(timestamp, field_to_goals);
//        }
//        outputToSmartDashboard();