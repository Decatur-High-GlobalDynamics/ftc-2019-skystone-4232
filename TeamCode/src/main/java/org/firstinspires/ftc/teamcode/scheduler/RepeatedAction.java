package org.firstinspires.ftc.teamcode.scheduler;

/**
 * RepeatedAction are for tasks that should be run each time through the Robot's loop(), but
 * which do not have a specific goal.
 *
 * Possibilities: IMU maintenance, RobotVision, GameControllers, etc
 */
public abstract class RepeatedAction extends OngoingAction{
    public RepeatedAction(String label, String descriptionFormat, Object...descriptionArgs){
        super(label, descriptionFormat, descriptionArgs);
    }

    public RepeatedAction(String label){
        this(label, null);
    }

    @Override
    public RepeatedAction start()
    {
        super.start();
        return this;
    }

    abstract protected void doTask();

    /**
     * this tries to make each loop an independent run... setting/clearing startTime and finishTime
     */
    @Override
    public final void loop()
    {
        startTime_ns = System.nanoTime();
        finishTime_ns= 0;

        //log("> Running");
        doTask();
        //log("< Done");

        // Each loop is a separate start time
        startTime_ns=0;
    }
}
