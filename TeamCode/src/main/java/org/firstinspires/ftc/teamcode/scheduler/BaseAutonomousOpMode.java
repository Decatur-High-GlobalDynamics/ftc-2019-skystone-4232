package org.firstinspires.ftc.teamcode.scheduler;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class BaseAutonomousOpMode extends BaseLinearOpMode
{
    public BaseAutonomousOpMode(Robot robot)
    {
        super(robot);
    }

    @Override
    public void teamInit() {
            super.teamInit();
            robot.setDrivingZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    abstract public void teamRun();
}
