package org.wfrobotics.reuse.subsystems.vision;
import java.util.ArrayList;
import java.util.List;

/** Parses message from vision coprocessor */
public class CoprocessorData
{
    private boolean valid;
    /** Time from capture at the coprocessor camera to sent on the socket */
    private double latencyCoprocessorSideMs;

    /** Pixels width the frame is*/
    public int frameWidthPixels;
    /** Pixels high the frame is*/
    public int frameHeightPixels;
    /** Frames per second, should be ~30 */
    public double fps;
    /** Time received at the RIO socket (in RIO time) adjusted for coprocessor total latency */
    public double atCameraTimestampMs;
    /** Items the GRIP pipeline identified in the frame */
    public List<VisionTargetInfo> targets;

    public CoprocessorData(String socketMsg, double rxTimestampMs)
    {
        targets = new ArrayList<VisionTargetInfo>();
        String[] parts = socketMsg.split(",");
        int index = 0;

        try
        {
            latencyCoprocessorSideMs = Double.parseDouble(parts[index++]);
            atCameraTimestampMs = rxTimestampMs - latencyCoprocessorSideMs;

            frameWidthPixels = Integer.parseInt(parts[index++]);
            frameHeightPixels = Integer.parseInt(parts[index++]);
            fps = Double.parseDouble(parts[index++]);
            int count = Integer.parseInt(parts[index++]);

            for(int i = 0; i < count; i++)
            {
                targets.add(new VisionTargetInfo(Double.parseDouble(parts[index++]), Double.parseDouble(parts[index++]), Integer.parseInt(parts[index++]), Integer.parseInt(parts[index++])));
            }
            valid = true;
        }
        catch (NumberFormatException e)
        {
            System.err.format("Cannot parse coprocessor data: %s\n", socketMsg);
            valid = false;
        }
    }

    /** Data received was able to be parsed out */
    public boolean isValid()
    {
        return valid;
    }

    public String toString()
    {
        return String.format("Pixels: %dx%d, FPS: %.1f, t: %.0fms, Latency: %.0fms, Targets: %d", frameWidthPixels, frameHeightPixels, fps, atCameraTimestampMs, latencyCoprocessorSideMs, targets.size());
    }

    /** Each identified target identified by the GRIP image processing pipeline */
    public class VisionTargetInfo
    {
        public final double center_x;
        public final double center_y;
        public final int width;
        public final int height;

        public VisionTargetInfo(double x, double y, int w, int h)
        {
            center_x = x;
            center_y = y;
            width = w;
            height = h;
        }

        public double getX()
        {
            return center_x;
        }

        public double getY()
        {
            return center_y;
        }

        public double getZ()
        {
            return 0.0; // TODO Compute based on area of year's target at given distance?
        }

        /** Pixels */
        public double area()
        {
            return width * height;
        }

        public String toString()
        {
            return String.format("Center: (x=%.1f, y=%.1f), Area: %.0f pixels", center_x, center_y, area());
        }
    }
}
