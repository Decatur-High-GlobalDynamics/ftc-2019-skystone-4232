package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeamRobot;
import org.firstinspires.ftc.teamcode.scheduler.BaseAutonomousOpMode;

@TeleOp(name = "T: EncoderTest", group = "Tinkering")
public class EncoderTestAutonomous extends  BaseAutonomousOpMode<TeamRobot>
{
    int startPosition, stopPosition;

    public EncoderTestAutonomous() {
        super(new TeamRobot());
    }

    @Override
    public void teamInit() {
        super.init();

        robot.setDrivingZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void teamRun()
    {
        startPosition = robot.getWheelPosition();
        stopPosition = startPosition + 4000;

        robot.driveStraight_raw(0.25);
        while ( shouldOpModeKeepRunning() && robot.getWheelPosition() < stopPosition )
        {}

        robot.startStopping().waitUntilFinished();

        // Keep telemetry going for a while
        teamSleep(15000, "Show telemetry");
    }
}
