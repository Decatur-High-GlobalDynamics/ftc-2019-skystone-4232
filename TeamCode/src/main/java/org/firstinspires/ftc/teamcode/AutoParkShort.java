package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.scheduler.BaseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.scheduler.EndableAction;
import org.firstinspires.ftc.teamcode.scheduler.Scheduler;
import org.firstinspires.ftc.teamcode.scheduler.TeamImu;

@Autonomous(name = "ParkingShort", group="Autonomous")
public class AutoParkShort extends BaseAutonomousOpMode<TeamRobot> {
    public AutoParkShort() {
        super(new TeamRobot());
    }

    @Override
    public void teamInit() {
        super.teamInit();
    }

    @Override
    public void teamRun() {
        try {
            Scheduler.get().sleep(10000, "Wait for other robot");
            robot.startInchMove(30, 0.5).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
        } catch (Exception e) {}
    }
}
