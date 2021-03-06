package org.firstinspires.ftc.teamcode.scheduler;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import static org.firstinspires.ftc.teamcode.scheduler.Utils.safeStringFormat;

public abstract class Action {
    static int actionCounterSequence= 0;
    final int actionID = ++actionCounterSequence;
    final Action parentAction;
    final Set<Action> childActions = new HashSet<>();

    Set<EndableAction> waitingToComplete = new HashSet<>();


    // How many parents/grandparents do we have (helps with indenting log)
    private final int ancestryCount;
    private final String messageIndentationString;

    String status = "";
    long statusTime_ns = System.nanoTime();

    final String label;
    final String fullDesc;
    final String ancesterLabels;
    long startTime_ns = 0;
    long finishTime_ns=0;
    Telemetry.Line telemetryStatusLine;


    public static Action getCurrentAction()
    {
        return Scheduler.get().currentAction;
    }

    public Action( String name )
    {
        this(name, null);
    }

    public Action(String name, String descriptionFormat, Object...descriptionArgs){
        this.parentAction = Scheduler.get().currentAction;

        if ( descriptionFormat != null ) {
            this.fullDesc = safeStringFormat(descriptionFormat, descriptionArgs);
        }
        else {
            this.fullDesc = name;
        }

        this.label = safeStringFormat("%s(#%d)", name, actionID);
        if (parentAction == null){
            ancesterLabels = label;
            ancestryCount=0;
            messageIndentationString="";
        }
        else {
            ancesterLabels = safeStringFormat("%s --> %s", parentAction.ancesterLabels, label);
            parentAction.childActionCreated(this);
            ancestryCount = parentAction.ancestryCount + 1;
            messageIndentationString = parentAction.messageIndentationString + "  ";
        }

        log("Created %s (%s)", fullDesc, ancesterLabels);
    }

    public Action start()
    {
        startTime_ns= System.nanoTime();
        finishTime_ns=0;
        Scheduler.get().actionStarted(this);
        startTelemetry();

        log("Started");

        return this;
    }

    protected void startTelemetry() {
        telemetryStatusLine = telemetryStatusLine = Scheduler.get().getTelemetry().addLine();
        telemetryStatusLine.addData("Action", new Func<Object>() {
            @Override
            public Object value() {
                return Robot.sharedInstance.saveTelemetryData("A-"+label, "%s -- %s", toShortString(), status);
            }
        });
    }


    public void log(String messageFormat, Object... args)
    {
        // Prefix the message with indentation and our label
        Utils.log_raw("<%s> %s", label, safeStringFormat(messageFormat, args));
    }

    public void setStatus(String statusFormat, Object... args)
    {
        String newStatus = safeStringFormat(statusFormat, args);

        // Any change in status?
        if (status.equals(newStatus))
            return;

        status = newStatus;
        log("Status changed after %.1f secs: %s",
                (System.nanoTime()-statusTime_ns)/1e9, status);

        statusTime_ns = System.nanoTime();
    }

    public int getAge_ms()
    {
        if ( startTime_ns == 0 )
            return -1;
        else if ( finishTime_ns > 0 )
            return (int) ((finishTime_ns - startTime_ns)/1e6);
        else
            return (int) ((System.nanoTime() - startTime_ns)/1e6);
    }

    private String getTimingString()
    {
        if ( startTime_ns == 0 )
            return "[not started]";
        else if ( finishTime_ns > 0 )
            return safeStringFormat("[dur=%.2f secs]", 1.0*(finishTime_ns - startTime_ns)/1e9);
        else
            return safeStringFormat("[age=%d secs]", (int)(1.0*(System.nanoTime() - startTime_ns)/1e9));

    }

    public String toShortString()
    {
        return safeStringFormat("%s%s", label, getTimingString());
    }

    public String toLongString()
    {
        return safeStringFormat("%d-%s|%s|%s|%s",
                actionID, fullDesc, getTimingString(), ancesterLabels, status);
    }

    @Override
    public String toString()
    {
        return toLongString();
    }

    public void finish()
    {
        finish(null);
    }

    public void finish(String messsageFormat, Object... messageArgs)
    {
        finishTime_ns = System.nanoTime();
        if ( messsageFormat == null )
            log("Finished");
        else
            log("Finished: " + messsageFormat, messageArgs );

        if ( parentAction != null )
        {
            parentAction.childActionFinished(this);
        }

        Scheduler.get().actionFinished(this);
    }


    protected void childActionCreated(Action childAction) {
        childActions.add(childAction);
    }

    protected void childActionFinished(Action childAction) {
        childActions.remove(childAction);
    }


    public boolean hasFinished()
    {
        if ( startTime_ns == 0 )
            return false;
        if ( finishTime_ns == 0 )
            return false;

        return true;
    }

    public boolean areChildrenDone(StringBuilder statusMessage)
    {
        boolean allChildrenAreDone = true;

        for(Action child : childActions)
        {
            if (!child.hasFinished())
            {
                allChildrenAreDone=false;
                if (statusMessage.length()==0)
                    statusMessage.append("Waiting for: ");
                else
                    statusMessage.append("|");

                statusMessage.append(child.label);
            }
        }
        return allChildrenAreDone;
    }


    public void actionSleep(final long sleep_ms, final String reason) throws InterruptedException
    {
        EndableAction sleepAction =
                new EndableAction("Sleep", "Sleep(%d ms, %s)", sleep_ms, reason)
                {
                    final long stopTime_ms = System.currentTimeMillis() + sleep_ms;

                    @Override
                    public boolean isDone(StringBuilder statusMessage)
                    {
                        statusMessage.append(safeStringFormat("%d secs of sleep left", (stopTime_ms- System.currentTimeMillis())/1000));
                        return System.currentTimeMillis()>=stopTime_ms;
                    }
                }.start();

        waitFor(sleepAction);
    }


    public void waitFor(EndableAction... actions) throws InterruptedException
    {
        String startingStatus = status;

        for(EndableAction a:actions)
        {
            if (a != null)
                waitingToComplete.add(a);
        }

        while (waitingToComplete.size() > 0)
        {
            // Keep the scheduler looping while we wait
            Scheduler.get().runLoopOnce();

            // Remove any completed actions
            Iterator<EndableAction> endableActionIterator = waitingToComplete.iterator();

            while (endableActionIterator.hasNext())
            {
                EndableAction endableAction = endableActionIterator.next();
                if (endableAction.hasFinished())
                    endableActionIterator.remove();
            }
        }
    }

    public void waitForChildren() throws InterruptedException
    {
        for (Action action : childActions)
        {
            if (action instanceof EndableAction)
            {
                waitFor((EndableAction) action);
            }
        }
    }

    public boolean isWaiting()
    {
        // Loop through a copy of our list of blocking Actions (so we can remove the ones that have finished0
        for(Action action : new ArrayList<>(waitingToComplete))
        {
            if (action.hasFinished())
                waitingToComplete.remove(action);
        }
        if (waitingToComplete.size()==0)
            return false;
        else
            return true;
    }

    protected void cleanup(boolean actionWasCompletedsSuccessfully)
    {
        if ( telemetryStatusLine != null )
        {
            Scheduler.get().getTelemetry().removeLine(telemetryStatusLine);
            Robot.sharedInstance.removeTelemetryData("A-" + label);
        }
    }



}
