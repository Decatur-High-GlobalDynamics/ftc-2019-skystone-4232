package org.firstinspires.ftc.teamcode.scheduler;

/**
 * RepeatedAction are for tasks that should be run each time through the Robot's runLoopOnce(), but
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

    @Override
    protected void startTelemetry()
    {
        // Not calling super.startTelemetry() so that RepeatedActions do not clutter up driver station
    }

    abstract protected void doTask();

    /**
     * this tries to make each runLoopOnce an independent run... setting/clearing startTime and finishTime
     */
    @Override
    public final void loop()
    {
        startTime_ns = System.nanoTime();
        finishTime_ns= 0;

        //log("> Running");
        doTask();
        //log("< Done");

        // Each runLoopOnce is a separate start time
        startTime_ns=0;
    }
}
