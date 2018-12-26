package org.wfrobotics.reuse.math.geometry;

import org.wfrobotics.reuse.math.CSVWritable;
import org.wfrobotics.reuse.math.interpolation.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
