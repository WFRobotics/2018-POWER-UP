package org.wfrobotics.reuse.commands.drive;

/** Turn until we see the target, or get to the expected heading it should be at **/
public class TurnUntilTargetInView extends TurnToHeading
{
    public TurnUntilTargetInView(double headingFieldRelative)
    {
        super(headingFieldRelative);
    }

    protected boolean isFinished()
    {
        if (state.visionInView)
        {
            return true;
        }
        return super.isFinished();
    }
}
