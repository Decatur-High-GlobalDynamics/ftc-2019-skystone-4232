package org.firstinspires.ftc.teamcode.scheduler;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
