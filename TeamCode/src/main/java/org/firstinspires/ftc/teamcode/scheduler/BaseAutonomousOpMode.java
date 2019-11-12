package org.firstinspires.ftc.teamcode.scheduler;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class BaseAutonomousOpMode<RobotClass extends Robot> extends BaseLinearOpMode<RobotClass>
{
    public BaseAutonomousOpMode(RobotClass robot)
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
