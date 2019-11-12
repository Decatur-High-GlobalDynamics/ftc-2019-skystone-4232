package org.firstinspires.ftc.teamcode.scheduler;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * OngoingAction represents an action that will be occurring for some time.
 * From when it is constructed, loop() will be called periodically, and
 * isUsingDcMotor(), isUsingServo() will be checked to make sure that only
 * one OngoingAction is controlling each robot component.
 *
 * Usually, OngoingAction is not used directly, but instead one of its subclasses
 * is used:
 *   EndableAction: Action that is temporary.
 */
public abstract class OngoingAction extends Action{
    // Managed by scheduler
        int numberOfLoops=0;
        long lastLoopDuration_ns;
        long lastLoopStart_ns=0;

    public OngoingAction(String label)
    {
        this(label, null);
    }

    public OngoingAction(String label, String descriptionFormat, Object...descriptionArgs){
        super(label, descriptionFormat, descriptionArgs);
    }

    // Only called by scheduler
    protected void loop()
    {
    }

    public boolean isUsingDcMotor(DcMotor motor)
    {
        return false;
    }

    public boolean isUsingServo(Servo servo)
    {
        return false;
    }
    public boolean isUsingServo(CRServo servo)
    {
        return false;
    }
}
