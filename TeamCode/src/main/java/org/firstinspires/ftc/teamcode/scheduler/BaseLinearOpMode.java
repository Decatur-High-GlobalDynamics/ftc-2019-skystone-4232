package org.firstinspires.ftc.teamcode.scheduler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.scheduler.Utils.log_raw;

abstract class BaseLinearOpMode<TeamRobotClass extends Robot> extends LinearOpMode implements SchedulerController
{
    public enum OPMODE_STATE {NEW, INITIALIZING, WAITING_FOR_START, RUNNING, STOPPING};

    private static final long TELEMETRY_LOGGING_INTERVAL_MS = 250L;

    final Scheduler scheduler = new Scheduler(this);
    protected final TeamRobotClass robot;

    OPMODE_STATE state = OPMODE_STATE.NEW;
    long opmodeStateChanged_ms = System.currentTimeMillis();


    protected Action opmodeAction;


    public BaseLinearOpMode(TeamRobotClass robot) {
        this.robot = robot;
    }

    // First thing to init
    final void baseInit()
    {
        // The first trackMovementSpeeds will correctly set the previous motor positions
        // The second trackMovementSpeeds will correctly set the speeds
        robot.trackMovementSpeeds();
        robot.trackMovementSpeeds();
        robot.setupRobotTelemetry(telemetry);

        telemetry.addLine()
                .addData("OpMode", new Func<Object>()
                {
                    @Override
                    public Object value()
                    {
                        return robot.saveTelemetryData("OpMode", "%s [%d secs].",
                                state, (System.currentTimeMillis() - opmodeStateChanged_ms) / 1000);
                    }
                });

        new RepeatedAction("RobotMaintenance")
        {
            @Override
            protected void doTask()
            {
                robot.trackMovementSpeeds();
                robot.protectRobot();
                updateTelemetry(telemetry);
            }
        }.start();
    }

    // Where OpModes can initialize themselves
    protected abstract void teamInit();

    // What happens after Play is pressed. OpMode completes when this returns.
    protected abstract void teamRun() throws InterruptedException;


    /**
     * Use this in loops to abort them when the OpMode should stop.
     * Note: This calls scheduler.runLoopOnce() to keep the robot running while your
     * while runLoopOnce waits for something
     *
     * for example: while (runSchedulerAndRobot() && distanceMoved<10) {//keep moving}
     *
     * @return Whether the OpMode should keep running
     */
    protected final boolean runSchedulerAndRobot() throws InterruptedException
    {
        // We run until stop is requested
        if (isStopRequested())
            return false;

        scheduler.runLoopOnce();

        return true;
    }


    /**
     * This is called from FIRST to start the OpMode
     * @throws InterruptedException
     */
    @Override
    public final void runOpMode() throws InterruptedException
    {
        robot.init(telemetry, hardwareMap, this);

        opmodeAction = new ImmediateAction("OpMode", "%s", getClass().getSimpleName());

        try
        {
            setOpModeState(OPMODE_STATE.INITIALIZING);
            opmodeAction.setStatus("Initializing: baseInit()");
            baseInit();
            opmodeAction.setStatus("Initializing: teamInit()");
            teamInit();

            setOpModeState(OPMODE_STATE.WAITING_FOR_START);
            opmodeAction.setStatus("Waiting for start");

            while (!isStarted())
                runSchedulerAndRobot();

            setOpModeState(OPMODE_STATE.RUNNING);
            opmodeAction.setStatus("Running: teamRun()");
            teamRun();
        }
        catch (StopRobotException e)
        {
            log_raw("TeamRobot stopping: %s", e.getMessage());
        }

        setOpModeState(OPMODE_STATE.STOPPING);
        opmodeAction.finish("runOpMode() is done");
    }

    private void setOpModeState(OPMODE_STATE newState)
    {
        opmodeAction.setStatus(newState.toString());
        state = newState;
        opmodeStateChanged_ms = System.currentTimeMillis();
    }

    public void teamSleep(long time_ms, String reason) throws InterruptedException
    {
        Scheduler.get().sleep(time_ms, reason);
    }

    public boolean shouldSchedulerStop()
    {
        if ( !isStarted() )
            return false;

        if ( isStopRequested() )
            return true;

        return false;
    }

    /**
     * Should be used only by the scheduler. Non-internal-scheduler needs should use teamSleep()
     * which calls Scheduler.sleep which then calls this.
     * @param time_ms
     */
    public void schedulerSleep(long time_ms) throws InterruptedException
    {
        long stopTime_ms = System.currentTimeMillis() + time_ms;
        while (!isStopRequested() && System.currentTimeMillis()<stopTime_ms)
        {
            idle();
            Thread.sleep(1);
        }
    }

    public Telemetry getTelemetry()
    {
        return telemetry;
    }

}
