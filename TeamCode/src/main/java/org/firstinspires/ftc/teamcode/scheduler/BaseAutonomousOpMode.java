package org.firstinspires.ftc.teamcode.scheduler;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract class BaseAutonomousOpMode extends BaseLinearOpMode
{
    public BaseAutonomousOpMode(Robot robot)
    {
        super(robot);
    }

    @Override
    void teamInit() {
            super.teamInit();
            robot.setDrivingZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    abstract public void teamRun();
}
