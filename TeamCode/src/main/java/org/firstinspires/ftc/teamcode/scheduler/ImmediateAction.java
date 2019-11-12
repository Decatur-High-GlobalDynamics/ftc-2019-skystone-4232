package org.firstinspires.ftc.teamcode.scheduler;

/**
 * ImmediateAction is a placeholder Action that is not used very much
 **/
public class ImmediateAction extends Action
{
    public ImmediateAction(String label)
    {
        this(label, null);
    }

    public ImmediateAction(String label, String descriptionFormat, Object... descriptionArgs)
    {
        super(label, descriptionFormat, descriptionArgs);

        // Immediate actions start right away
        start();
    }
}
