package org.wfrobotics.reuse.config;

import org.wfrobotics.reuse.math.geometry.Pose2d;
import org.wfrobotics.reuse.subsystems.control.Path;

/**
 * Holds a {@link Path}, allows modification to the path
 *
 * <p>Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 * @author Code: Team 254<br>Documentation: STEM Alliance of Fargo Moorhead
 */
public interface PathContainer {
    Path buildPath();

    Pose2d getStartPose();

    boolean isReversed();
}
