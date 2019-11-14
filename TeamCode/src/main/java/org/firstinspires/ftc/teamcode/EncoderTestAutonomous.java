package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.scheduler.BaseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.scheduler.Scheduler;

@Autonomous(name = "T: EncoderTest", group = "Tinkering")
public class EncoderTestAutonomous extends BaseAutonomousOpMode<TeamRobot>
{
    int startPosition, stopPosition;

    public EncoderTestAutonomous() {
        super(new TeamRobot());
    }

    @Override
    public void teamInit() {
        robot.setDrivingZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void teamRun() throws InterruptedException
    {
        int distance=4000;

        startPosition = robot.getWheelPosition();
        stopPosition = startPosition + distance;

        opmodeAction.setStatus("Starting to move forward");
        robot.driveStraight_raw(0.25);
        while ( runSchedulerAndRobot() && robot.getWheelPosition() < stopPosition )
        {
            opmodeAction.setStatus("Moving forward for %d encoder clicks (%d left to go)",
                    distance,
                    stopPosition-robot.getWheelPosition());
        }

        opmodeAction.setStatus("Done. Moved %d encoder clicks", robot.getWheelPosition()-startPosition);

        robot.startStopping().waitUntilFinished();

        // Keep telemetry going for a while
        teamSleep(15000, "Show telemetry");
    }
}
