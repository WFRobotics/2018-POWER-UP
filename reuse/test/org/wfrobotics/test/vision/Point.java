package org.wfrobotics.test.vision;

import org.wfrobotics.reuse.subsystems.vision.CoprocessorData.VisionTargetInfo;

public class Point
{
    double time;
    double x_error;
    double y_error;
    VisionTargetInfo target;


    public Point(double time, VisionTargetInfo target)
    {
        x_error = target.getX();
        this.target = target;
        this.time = time;
    }
    public Point(double time, double x_error, double y_error)
    {
        this.time = time;
        this.x_error = x_error;
        this.y_error = y_error;
    }
    public Point extrapolate(Point other, double time) {
        Point ex_pt = new Point(time, x_error * (other.x_error - x_error) + x_error, y_error * (other.y_error - y_error) + y_error  );
        return ex_pt;
        //        x_error * (other.x_error - x_error) + x_error, 0, target.width, target.height
        //        return new Translation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }
}
