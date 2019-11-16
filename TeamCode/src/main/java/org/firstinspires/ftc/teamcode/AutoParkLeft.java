package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.scheduler.BaseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.scheduler.EndableAction;
import org.firstinspires.ftc.teamcode.scheduler.TeamImu;

@Autonomous(name = "ParkingLeft", group="Autonomous")
public class AutoParkLeft extends BaseAutonomousOpMode<TeamRobot> {
    public AutoParkLeft() {
        super(new TeamRobot());
    }

    @Override
    public void teamInit() {
        super.teamInit();
    }

    @Override
    public void teamRun() {
        try {
            robot.startInchMove(10, 0.5).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startTurningLeft(90);
            robot.startInchMove(24, 0.5).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
        } catch (Exception e) {}
    }
}
